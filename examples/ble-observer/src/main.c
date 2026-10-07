/*
 * ble-observer — passive Tapestry BLE observer (nRF5340 DK).
 *
 * Scans continuously (passive, window == interval, no duplicate
 * filtering, legacy + extended reports) and writes one line per Tapestry
 * advertisement to the console UART:
 *
 *   t_us,rssi,addr,adv,kind,id,seq,hex
 *
 *   t_us  DK uptime in microseconds when the host stack delivered the
 *         report (one clock for every robot)
 *   rssi  dBm
 *   addr  advertiser address (a per-boot random address on robots; the
 *         gossip and debug sets of one robot have different ones)
 *   adv   L legacy, E extended; a trailing ! marks an incomplete report
 *   kind  G gossip, B ID-negotiation beacon (gossip frame, id 0xFF),
 *         F shared-fact frame (wire v7), S debug status, T debug text,
 *         U any other Tapestry AD
 *   id    element id (255 while negotiating); F: the sender's id
 *   seq   G: update_seq; B: the nonce; F: the table digest;
 *         S/T: the debug packet seq; U: 0
 *   hex   the manufacturer data after the company id, starting with the
 *         type/kind byte — decoded on the host (tools/observer_log.py)
 *
 * Other lines start with '#': a banner at boot and a stats line every
 * second.  Formatting happens in the BT RX thread; a low-priority thread
 * drains a ring buffer to the UART, so a slow port costs dropped lines
 * (counted in the stats line), never scan time.
 */

#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/atomic.h>

#include <tapestry/wire.h>
#include <tapestry/csm.h>

/* Bump when the line layout above changes; the host checks it. */
#define LINE_FORMAT_VERSION 1

#define DEBUG_KIND_STATUS 0x10u
#define DEBUG_KIND_TEXT   0x11u
#define DEBUG_PAYLOAD_LEN 27u    /* examples/cutebot-formation/src/ble_debug.h */

#define STATS_PERIOD_MS 1000

static const struct device *const uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

RING_BUF_DECLARE(s_out, 16384);
static struct k_spinlock s_out_lock;
static K_SEM_DEFINE(s_out_sem, 0, 1);

static atomic_t s_reports;       /* every advertising report         */
static atomic_t s_tapestry;      /* reports carrying a Tapestry AD   */
static atomic_t s_incomplete;    /* ...of which were incomplete      */
static atomic_t s_dropped;       /* lines lost to a full ring buffer */

static void emit(const char *line, size_t len)
{
	k_spinlock_key_t key = k_spin_lock(&s_out_lock);

	if (ring_buf_space_get(&s_out) >= len) {
		ring_buf_put(&s_out, (const uint8_t *)line, len);
	} else {
		atomic_inc(&s_dropped);
	}
	k_spin_unlock(&s_out_lock, key);
	k_sem_give(&s_out_sem);
}

static void writer_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	uint8_t chunk[64];

	for (;;) {
		k_sem_take(&s_out_sem, K_FOREVER);
		for (;;) {
			k_spinlock_key_t key = k_spin_lock(&s_out_lock);
			uint32_t n = ring_buf_get(&s_out, chunk, sizeof(chunk));

			k_spin_unlock(&s_out_lock, key);
			if (n == 0) {
				break;
			}
			for (uint32_t i = 0; i < n; i++) {
				uart_poll_out(uart, chunk[i]);
			}
		}
	}
}

K_THREAD_DEFINE(s_writer, 1024, writer_fn, NULL, NULL, NULL,
		K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

struct mfr_ctx {
	const uint8_t *data;   /* after the company id */
	uint8_t len;
};

static bool find_tapestry_ad(struct bt_data *ad, void *user_data)
{
	struct mfr_ctx *ctx = user_data;

	if (ad->type != BT_DATA_MANUFACTURER_DATA || ad->data_len < 3u ||
	    ad->data[0] != TAPESTRY_BLE_COMPANY_ID_LO ||
	    ad->data[1] != TAPESTRY_BLE_COMPANY_ID_HI) {
		return true;
	}
	ctx->data = ad->data + 2;
	ctx->len = ad->data_len - 2u;
	return false;
}

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	uint64_t t_us = k_ticks_to_us_floor64(k_uptime_ticks());
	struct mfr_ctx ctx = {0};

	atomic_inc(&s_reports);
	bt_data_parse(buf, find_tapestry_ad, &ctx);
	if (ctx.data == NULL) {
		return;
	}
	atomic_inc(&s_tapestry);

	bool ext = (info->adv_props & BT_GAP_ADV_PROP_EXT_ADV) != 0;
	bool incomplete = BT_HCI_LE_ADV_EVT_TYPE_DATA_STATUS(info->adv_props) !=
			  BT_HCI_LE_ADV_EVT_TYPE_DATA_STATUS_COMPLETE;
	if (incomplete) {
		atomic_inc(&s_incomplete);
	}

	const uint8_t *p = ctx.data;
	char kind = 'U';
	uint8_t id = 0;
	uint32_t seq = 0;

	if (p[0] == TAPESTRY_MSG_GOSSIP && ctx.len >= 1u + TAPESTRY_GOSSIP_FRAME_SIZE) {
		const uint8_t *f = p + 1;

		id = f[offsetof(tapestry_gossip_frame_t, id)];
		seq = sys_get_le32(&f[offsetof(tapestry_gossip_frame_t, update_seq)]);
		kind = (id == ELEMENT_ID_INVALID) ? 'B' : 'G';
	} else if (p[0] == TAPESTRY_MSG_FACT &&
		   ctx.len >= 1u + TAPESTRY_FACT_FRAME_SIZE) {
		const uint8_t *f = p + 1;

		id = f[offsetof(tapestry_fact_frame_t, sender_id)];
		seq = sys_get_le16(&f[offsetof(tapestry_fact_frame_t, digest)]);
		kind = 'F';
	} else if ((p[0] == DEBUG_KIND_STATUS || p[0] == DEBUG_KIND_TEXT) &&
		   ctx.len == DEBUG_PAYLOAD_LEN) {
		id = p[1];
		seq = sys_get_le16(&p[2]);
		kind = (p[0] == DEBUG_KIND_STATUS) ? 'S' : 'T';
	}

	char addr[BT_ADDR_STR_LEN];
	char line[96 + 2 * 255];

	bt_addr_to_str(&info->addr->a, addr, sizeof(addr));
	int n = snprintk(line, sizeof(line), "%llu,%d,%s,%s,%c,%u,%u,",
			 (unsigned long long)t_us, info->rssi, addr,
			 ext ? (incomplete ? "E!" : "E") : (incomplete ? "L!" : "L"),
			 kind, id, seq);

	static const char hex[] = "0123456789abcdef";
	for (uint8_t i = 0; i < ctx.len && n < (int)sizeof(line) - 2; i++) {
		line[n++] = hex[p[i] >> 4];
		line[n++] = hex[p[i] & 0xFu];
	}
	line[n++] = '\n';
	emit(line, (size_t)n);
}

static struct bt_le_scan_cb scan_cb = {
	.recv = scan_recv,
};

static void emitf(const char *fmt, ...)
{
	char line[160];
	va_list ap;

	va_start(ap, fmt);
	int n = vsnprintk(line, sizeof(line), fmt, ap);
	va_end(ap);
	if (n > 0) {
		emit(line, MIN((size_t)n, sizeof(line) - 1));
	}
}

int main(void)
{
	emitf("# tapestry-ble-observer fmt=%d wire=%u\n", LINE_FORMAT_VERSION,
	      TAPESTRY_WIRE_VERSION);
	emitf("# t_us,rssi,addr,adv,kind,id,seq,hex\n");

	if (led.port != NULL && gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	}

	int err = bt_enable(NULL);
	if (err) {
		emitf("# error bt_enable %d\n", err);
		return 0;
	}

	bt_le_scan_cb_register(&scan_cb);

	/* Continuous: the scanner never leaves the air, it only rotates the
	 * primary channel every interval.  No duplicate filtering — every
	 * repeat is a sample of the link. */
	const struct bt_le_scan_param param = {
		.type     = BT_LE_SCAN_TYPE_PASSIVE,
		.options  = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window   = BT_GAP_SCAN_FAST_INTERVAL,
	};

	err = bt_le_scan_start(&param, NULL);
	if (err) {
		emitf("# error bt_le_scan_start %d\n", err);
		return 0;
	}
	emitf("# scanning\n");

	for (;;) {
		k_msleep(STATS_PERIOD_MS);
		emitf("# stats t_us=%llu reports=%ld tapestry=%ld incomplete=%ld "
		      "dropped=%ld\n",
		      (unsigned long long)k_ticks_to_us_floor64(k_uptime_ticks()),
		      atomic_set(&s_reports, 0), atomic_set(&s_tapestry, 0),
		      atomic_set(&s_incomplete, 0), atomic_get(&s_dropped));
		if (led.port != NULL) {
			gpio_pin_toggle_dt(&led);
		}
	}
	return 0;
}
