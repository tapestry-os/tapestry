# Vendored Zephyr patches

These patches fix bugs in the vendored `zephyr/` tree (a west module, not
part of this repo) that are required for Tapestry hardware to work
correctly. `west update` can silently discard them, so they must be
reapplied after any `west update` that touches `zephyr/`.

## zephyr-i2c-stm32-v1-rtio-bmi08x-fixes.patch

Fixes two real bugs in `drivers/i2c/i2c_stm32_v1_rtio.c` (STM32 I2C v1 RTIO
driver) discovered while bringing up the BMI088 gyro over I2C3 on
crazyflie21br:

1. A spurious second RXNE interrupt after a single-byte master read
   double-completes the RTIO transaction and crashes (`txn_head == NULL`
   guard in `i2c_stm32_controller_mode_end()`).
2. A spurious BERR (Bus Error) flag can be latched mid-transfer with no
   actual misplaced start/stop condition, previously treated as fatal
   (`-EIO`), which broke the BMI088 gyro soft-reset write ("Cannot reboot
   chip"). Now cleared and ignored, letting the transfer complete normally.

Reapply after `west update`:

```bash
cd zephyr
git apply ../tapestry/patches/zephyr-i2c-stm32-v1-rtio-bmi08x-fixes.patch
```
