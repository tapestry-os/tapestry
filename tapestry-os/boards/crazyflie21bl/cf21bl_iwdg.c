/*
 * cf21bl_iwdg.c — early IWDG kick for Crazyflie 2.1 brushless
 *
 * Runs at PRE_KERNEL_1 priority 0: before any Zephyr driver.
 *
 * The Bitcraze bootloader arms the IWDG before jumping to us.  Without an
 * early kick, Zephyr would reset before finishing boot.  We extend the
 * timeout to ~32 s so the application has time to start its own kicks.
 *
 * Root-cause history:
 *
 * Bug 1: the original while(IWDG_SR & 0x3) loop hung forever because LSI
 * was not running.  Writes to IWDG_PR/RLR set PVU/RVU, which only clear
 * once LSI synchronises the new values.  Fix: force LSI on (write 0xCCCC to
 * IWDG_KR, which RM0090 §21.3 guarantees forces LSI on, then set LSION and
 * wait for LSIRDY with a bail-out).
 *
 * Bug 2: RM0090 §21.3 says writes to IWDG_PR/RLR are silently ignored when
 * PVU/RVU are already set.  If the bootloader wrote IWDG_PR/RLR just before
 * jumping to us, our writes land while PVU/RVU are still set → they are
 * discarded → the bootloader's short (~183 ms) timeout remains → IWDG fires
 * during our diagnostic blink_pd2 loop → board resets → no visible blink.
 * Fix: wait for PVU=0 before writing PR, and RVU=0 before writing RLR.
 *
 * STM32F405 IWDG registers (RM0090 §21):
 *   KR  0x40003000  write 0x5555=unlock, 0xAAAA=kick, 0xCCCC=start+force-LSI
 *   PR  0x40003004  prescaler: 6 → /256 → ~8 ms/count at LSI 32 kHz
 *   RLR 0x40003008  reload: 0xFFF (4095) → ~32.8 s timeout
 *   SR  0x4000300C  bit0=PVU, bit1=RVU — must be 0 before writing PR/RLR
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cf21bl_iwdg, LOG_LEVEL_INF);

#define IWDG_KR   (*(volatile uint32_t *)0x40003000U)
#define IWDG_PR   (*(volatile uint32_t *)0x40003004U)
#define IWDG_RLR  (*(volatile uint32_t *)0x40003008U)
#define IWDG_SR   (*(volatile uint32_t *)0x4000300CU)

/* RCC_CSR: LSION=bit0, LSIRDY=bit1 */
#define RCC_CSR  (*(volatile uint32_t *)0x40023874U)

static int cf21bl_iwdg_init(void)
{
    /* Step 1: force LSI on.
     * Writing 0xCCCC to IWDG_KR starts the IWDG and forces LSI on (RM0090
     * §21.3: "the LSI oscillator is forced on and cannot be disabled").
     * Also set LSION in RCC_CSR as belt-and-suspenders.
     * LSI startup ≤ 250 µs; 200 000 iterations is > 1 ms at any boot clock. */
    IWDG_KR = 0xCCCCU;
    RCC_CSR |= 1U;
    volatile int t = 200000;
    while (!(RCC_CSR & 2U) && --t) {}         /* wait LSIRDY, with bail-out */

    /* Step 2: kick immediately with the bootloader's current timeout to
     * reset the counter and buy the full window for reprogramming. */
    IWDG_KR = 0xAAAAU;

    /* Step 3: reprogram PR and RLR.
     *
     * CRITICAL: writes to IWDG_PR/RLR are silently discarded when PVU/RVU
     * are set (RM0090: "value can be updated only when bit is reset").
     * Wait for PVU=0 before writing PR, and RVU=0 before writing RLR.
     * With LSI running, each bit clears in ≤ 4 LSI cycles ≈ 125 µs, which
     * is well within our 200 000-iteration bail-out (> 1 ms). */
    IWDG_KR = 0x5555U;                        /* unlock PR and RLR */

    t = 200000;
    while ((IWDG_SR & 0x1U) && --t) {}        /* wait PVU=0 before PR write */
    IWDG_PR = 6U;                              /* /256 */

    t = 200000;
    while ((IWDG_SR & 0x2U) && --t) {}        /* wait RVU=0 before RLR write */
    IWDG_RLR = 0xFFFU;                        /* 4095 counts → ~32.8 s */

    t = 200000;
    while ((IWDG_SR & 0x3U) && --t) {}        /* wait for both to latch */

    /* Step 4: kick — counter now reloads from the new ~32 s shadow. */
    IWDG_KR = 0xAAAAU;

    return 0;
}

static void cf21bl_iwdg_kick_fn(struct k_timer *t)
{
    ARG_UNUSED(t);
    IWDG_KR = 0xAAAAU;

#ifdef CONFIG_CF21BL_IWDG_HEARTBEAT_LED
    /* Diagnostic heartbeat: toggle the status LED on every kick (10 s).
     * This handler runs at interrupt level via the system clock, so the
     * LED keeps toggling through any thread-level hang or deadlock —
     * a frozen LED during a silent-console hang means interrupts
     * themselves are starved (IRQ lock / ISR storm), while a still-
     * toggling LED means the kernel is alive and the silence is a
     * logging/console failure.  Registers poked directly: this must not
     * depend on any driver state. */
    #define HB_GPIOC_ODR (*(volatile uint32_t *)0x40020814U)
    HB_GPIOC_ODR ^= (1U << 1);   /* PC1 = status LED (active-low) */
#endif
}

static K_TIMER_DEFINE(cf21bl_iwdg_timer, cf21bl_iwdg_kick_fn, NULL);

static int cf21bl_iwdg_start_timer(void)
{
    /* Log why the previous reset happened (RM0090 §6.3.21 RCC_CSR flags),
     * then clear the flags (RMVF) so the next boot reports only its own
     * cause.  IWDG = watchdog starved (IRQ lock / ISR storm upstream);
     * PIN without IWDG = external reset (NRST — e.g. the nRF51 power-
     * cycling the STM32); SFT = software reset (Zephyr fatal handler). */
    uint32_t csr = RCC_CSR;
    LOG_INF("reset cause: CSR=0x%08x —%s%s%s%s%s%s%s",
            csr,
            (csr & (1U << 31)) ? " LPWR" : "",
            (csr & (1U << 30)) ? " WWDG" : "",
            (csr & (1U << 29)) ? " IWDG" : "",
            (csr & (1U << 28)) ? " SFT"  : "",
            (csr & (1U << 27)) ? " POR"  : "",
            (csr & (1U << 26)) ? " PIN"  : "",
            (csr & (1U << 25)) ? " BOR"  : "");
    RCC_CSR |= (1U << 24);   /* RMVF: clear reset flags */

    /* Kick every 10 s — well within the ~32 s timeout. */
    k_timer_start(&cf21bl_iwdg_timer, K_SECONDS(10), K_SECONDS(10));
    return 0;
}

SYS_INIT(cf21bl_iwdg_init,         PRE_KERNEL_1, 0);
SYS_INIT(cf21bl_iwdg_start_timer,  APPLICATION,  0);
