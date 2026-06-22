/*
 * AeroDrag Wheel — monitoraggio batteria LiPo del XIAO BLE Sense.
 *
 * Sul XIAO nRF52840 la tensione di batteria si legge su un partitore:
 *   - pin di enable del partitore (P0.14, ATTIVO BASSO) → io abilitato dall'overlay
 *   - canale ADC su AIN7 (P0.31)                        → io-channels in zephyr,user
 *   - fattore partitore ≈ 2.96  (R 1M + 0.51M)
 *
 * Best-effort: se l'overlay non definisce l'ADC, le funzioni ritornano errore e
 * il resto del firmware funziona comunque (la batteria è una feature di prodotto,
 * non parte del contratto 0xBB00).
 *
 * NB bring-up: VERIFICARE pin/partitore sulla revisione esatta della board.
 */
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include "wheel.h"

LOG_MODULE_REGISTER(wheel_batt, LOG_LEVEL_INF);

/* Abilita il codice ADC solo se l'overlay fornisce il nodo zephyr,user con
 * io-channels (così il build non rompe su board senza la definizione). */
#if DT_NODE_EXISTS(DT_PATH(zephyr_user)) && DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>

#define VBAT_DIVIDER 2.96f          /* (1M + 0.51M) / 0.51M                    */
#define LIPO_FULL_MV 4200.0f
#define LIPO_EMPTY_MV 3300.0f

static const struct adc_dt_spec vbat_adc = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
static const struct gpio_dt_spec vbat_en =
	GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), vbat_enable_gpios, {0});

static bool s_ready;

int wheel_battery_init(void)
{
	if (!adc_is_ready_dt(&vbat_adc)) {
		LOG_WRN("ADC non pronto — batteria disabilitata");
		return -ENODEV;
	}
	if (adc_channel_setup_dt(&vbat_adc) < 0) {
		LOG_WRN("adc_channel_setup fallito — batteria disabilitata");
		return -EIO;
	}
	if (vbat_en.port && !gpio_is_ready_dt(&vbat_en)) {
		LOG_WRN("gpio enable batteria non pronto");
	}
	if (vbat_en.port) {
		gpio_pin_configure_dt(&vbat_en, GPIO_OUTPUT_INACTIVE);
	}
	s_ready = true;
	LOG_INF("batteria pronta (AIN VBAT)");
	return 0;
}

static uint8_t mv_to_pct(float mv)
{
	if (mv >= LIPO_FULL_MV) {
		return 100;
	}
	if (mv <= LIPO_EMPTY_MV) {
		return 0;
	}
	/* Curva LiPo approssimata (più fedele di una retta sui bordi). */
	float pct = (mv - LIPO_EMPTY_MV) / (LIPO_FULL_MV - LIPO_EMPTY_MV) * 100.0f;

	return (uint8_t)(pct + 0.5f);
}

int wheel_battery_read_pct(uint8_t *pct_out)
{
	int16_t raw = 0;
	struct adc_sequence seq = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};

	if (!s_ready || pct_out == NULL) {
		return -ENODEV;
	}

	if (vbat_en.port) {
		gpio_pin_set_dt(&vbat_en, 1);   /* attivo (basso fisico) → abilita partitore */
		k_sleep(K_MSEC(5));
	}

	(void)adc_sequence_init_dt(&vbat_adc, &seq);
	int rc = adc_read_dt(&vbat_adc, &seq);

	if (vbat_en.port) {
		gpio_pin_set_dt(&vbat_en, 0);   /* disabilita partitore (risparmio) */
	}

	if (rc < 0) {
		LOG_WRN("adc_read fallito (%d)", rc);
		return rc;
	}

	int32_t mv = raw;

	if (adc_raw_to_millivolts_dt(&vbat_adc, &mv) < 0) {
		return -EIO;
	}

	float vbat_mv = (float)mv * VBAT_DIVIDER;

	*pct_out = mv_to_pct(vbat_mv);
	LOG_DBG("vbat=%d mV → %u%%", (int)vbat_mv, *pct_out);
	return 0;
}

#else /* nessun ADC definito nell'overlay */

int wheel_battery_init(void)
{
	LOG_INF("batteria non configurata (nessun io-channels in zephyr,user)");
	return -ENOTSUP;
}

int wheel_battery_read_pct(uint8_t *pct_out)
{
	ARG_UNUSED(pct_out);
	return -ENOTSUP;
}

#endif
