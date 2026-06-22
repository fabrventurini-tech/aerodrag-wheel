/*
 * AeroDrag Wheel — firmware sensore ruota Crr (coast-down).
 * Board: Seeed XIAO BLE Sense (nRF52840) — IMU onboard LSM6DS3TR-C (I2C) + LiPo.
 * Toolchain: nRF Connect SDK / Zephyr.   Build: west build -b xiao_ble/nrf52840/sense
 *
 * Espone il servizio BLE 0xBB00 (contract firmware↔wheel, §2.G). Consumer = ESP32
 * (BLE central) che relaya lo stream all'app (0xaa0c), inoltra i comandi coast-down
 * (0xaa0d→0xBB03) e scrive la config (0xaa08→0xBB04).
 *
 * ── MODELLO FISICO (montaggio sul MOZZO) ─────────────────────────────────────
 *   omega   = |gyro[asse_spin]|             [rad/s]  (auto-rilevato, vedi sotto)
 *   radius  = tireCircM / (2·pi)            [m]
 *   speedMs = omega · radius                [m/s]
 *   accelMs2 = LP( d(speedMs)/dt )          [m/s²]  (decel coast-down = NEGATIVA)
 *   vibRMS  = RMS( |accel| - media_mobile ) [m/s²]  (rugosità superficie)
 *   tempC   = temperatura die IMU           [°C]
 *
 * CRITICO: il giroscopio va portato a FONDO-SCALA ±2000 dps (≈34.9 rad/s). Col
 * default ±245 dps saturerebbe a ~3 km/h e la misura sarebbe inutilizzabile.
 * A ±2000 dps il tetto è ~42 km/h su ruota 700c (omega·r): i coast-down partono
 * tipicamente ≤35 km/h. Vedi README per i dettagli.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>
#include <errno.h>

#include "wheel.h"

#if IS_ENABLED(CONFIG_BT_BAS)
#include <zephyr/bluetooth/services/bas.h>
#endif

LOG_MODULE_REGISTER(aerodrag_wheel, LOG_LEVEL_INF);

#define PI_F             3.14159265f

/* Cadenze: 100 Hz in coast-down (campionamento per accel/vib), 5 Hz in idle
 * (solo per restare reattivi; in idle non si trasmette → risparmio batteria). */
#define SAMPLE_HZ_ACTIVE 100
#define STREAM_DIV       10                       /* notify a 100/10 = 10 Hz   */
#define PERIOD_ACTIVE    K_MSEC(1000 / SAMPLE_HZ_ACTIVE)
#define PERIOD_IDLE      K_MSEC(200)
#define DT_S             (1.0f / SAMPLE_HZ_ACTIVE)

/* Filtri */
#define OMEGA_LP_ALPHA   0.40f                    /* ~8 Hz: stabilizza la vel.  */
#define ACCEL_LP_ALPHA   0.12f                    /* ~2 Hz: come da spec app    */
#define VIB_MEAN_ALPHA   0.05f                    /* media mobile |accel|       */

/* Auto-rilevamento asse di spin */
#define AXIS_EMA_ALPHA   0.10f
#define AXIS_LOCK_RADS   5.0f                     /* ~1.7 m/s sul mozzo         */

/* IMU full-scale / ODR */
#define IMU_ODR_HZ       208
#define GYRO_FS_RADS_V1  34                       /* 34.906585 rad/s = 2000 dps */
#define GYRO_FS_RADS_V2  906585
#define ACCEL_FS_MS2     157                      /* ≈ ±16 g                    */

/* Sicurezza: auto-annulla un coast-down che dura troppo (app sparita) */
#define MAX_RUN_MS       180000                   /* 3 min                      */

/* Batteria */
#define BATT_PERIOD      K_SECONDS(60)

/* ── IMU node (alias 'imu' o primo LSM6DSL/LSM6DSO okay della DTS) ──────────── */
#if DT_NODE_EXISTS(DT_ALIAS(imu))
#define IMU_NODE DT_ALIAS(imu)
#elif DT_HAS_COMPAT_STATUS_OKAY(st_lsm6dsl)
#define IMU_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(st_lsm6dsl)
#elif DT_HAS_COMPAT_STATUS_OKAY(st_lsm6dso)
#define IMU_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(st_lsm6dso)
#else
#error "IMU non trovata: definisci un alias 'imu' o abilita l'IMU onboard nella DTS"
#endif

/* ── Stato ─────────────────────────────────────────────────────────────────── */
static const struct device *g_imu;

static struct wheel_config g_cfg = {
	.tire_circ_m = WHEEL_TIRE_CIRC_DEFAULT,
	.mass_kg     = WHEEL_MASS_DEFAULT,
};
static volatile float g_radius_m = WHEEL_TIRE_CIRC_DEFAULT / (2.0f * PI_F);
static volatile enum coast_mode g_mode = COAST_IDLE;

static float clampf(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

/* ── API core invocate dal layer BLE ───────────────────────────────────────── */

int wheel_core_apply_cmd(uint8_t cmd)
{
	switch (cmd) {
	case COAST_CMD_INDOOR: g_mode = COAST_INDOOR; break;
	case COAST_CMD_OUT_A:  g_mode = COAST_OUT_A;  break;
	case COAST_CMD_OUT_B:  g_mode = COAST_OUT_B;  break;
	case COAST_CMD_CANCEL: g_mode = COAST_IDLE;   break;
	default:
		return -EINVAL;
	}
	LOG_INF("coast cmd=0x%02X → mode=%d", cmd, (int)g_mode);
	return 0;
}

void wheel_core_set_config(const struct wheel_config *cfg)
{
	g_cfg.tire_circ_m = clampf(cfg->tire_circ_m, WHEEL_TIRE_CIRC_MIN, WHEEL_TIRE_CIRC_MAX);
	g_cfg.mass_kg     = clampf(cfg->mass_kg, WHEEL_MASS_MIN, WHEEL_MASS_MAX);
	g_radius_m = g_cfg.tire_circ_m / (2.0f * PI_F);
	LOG_INF("config: circ=%.3f m mass=%.1f kg (r=%.4f m)",
		(double)g_cfg.tire_circ_m, (double)g_cfg.mass_kg, (double)g_radius_m);
	wheel_store_save(&g_cfg);
}

void wheel_core_get_config(struct wheel_config *out)
{
	*out = g_cfg;
}

/* ── Configurazione IMU (ODR + fondo-scala) ────────────────────────────────── */
static void imu_configure(const struct device *dev)
{
	struct sensor_value v;
	int rc;

	/* ODR accel + gyro */
	v.val1 = IMU_ODR_HZ; v.val2 = 0;
	rc = sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
	if (rc) LOG_WRN("set accel ODR rc=%d (può essere impostato da DTS)", rc);
	v.val1 = IMU_ODR_HZ; v.val2 = 0;
	rc = sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
	if (rc) LOG_WRN("set gyro ODR rc=%d", rc);

	/* Fondo-scala accel ≈ ±16 g */
	v.val1 = ACCEL_FS_MS2; v.val2 = 0;
	rc = sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
	if (rc) LOG_WRN("set accel FS rc=%d", rc);

	/* Fondo-scala gyro ±2000 dps — IL FIX CRITICO */
	v.val1 = GYRO_FS_RADS_V1; v.val2 = GYRO_FS_RADS_V2;
	rc = sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
	if (rc) {
		LOG_ERR("set gyro FS ±2000dps rc=%d — VERIFICARE: il default ±245dps "
			"satura a ~3 km/h!", rc);
	} else {
		LOG_INF("IMU: ODR=%d Hz, gyro ±2000 dps, accel ±16 g", IMU_ODR_HZ);
	}
}

/* ── Campionamento + calcolo + notify ──────────────────────────────────────── */

/* Stato run (resettato a ogni transizione IDLE→attivo) */
static float g_speed_ms;
static float g_accel_lp;
static float g_omega_lp;
static float g_am_mean;
static float g_vib_acc;
static uint16_t g_vib_n;
static float g_temp_c = 20.0f;
static uint8_t g_div;
static bool g_primed;
static float g_axis_ema[3];
static int  g_spin_axis = 2;     /* default Z, poi auto-rilevato */
static bool g_spin_locked;

static void run_reset(void)
{
	g_speed_ms = 0; g_accel_lp = 0; g_omega_lp = 0;
	g_am_mean = 0; g_vib_acc = 0; g_vib_n = 0;
	g_div = 0; g_primed = false;
	g_axis_ema[0] = g_axis_ema[1] = g_axis_ema[2] = 0;
	g_spin_locked = false; g_spin_axis = 2;
}

static void process_sample(void)
{
	struct sensor_value gyro[3], accel[3], temp;

	if (sensor_sample_fetch(g_imu) != 0) {
		return;
	}

	/* --- velocità angolare ruota → auto-asse di spin → velocità/accel --- */
	if (sensor_channel_get(g_imu, SENSOR_CHAN_GYRO_XYZ, gyro) == 0) {
		for (int i = 0; i < 3; i++) {
			float r = fabsf((float)sensor_value_to_double(&gyro[i]));
			g_axis_ema[i] += AXIS_EMA_ALPHA * (r - g_axis_ema[i]);
		}
		if (!g_spin_locked) {
			int mx = 0;
			for (int i = 1; i < 3; i++) {
				if (g_axis_ema[i] > g_axis_ema[mx]) mx = i;
			}
			if (g_axis_ema[mx] > AXIS_LOCK_RADS) {
				g_spin_axis = mx;
				g_spin_locked = true;
				LOG_INF("asse spin rilevato: %c", "XYZ"[mx]);
			}
		}

		float omega_raw = fabsf((float)sensor_value_to_double(&gyro[g_spin_axis]));
		g_omega_lp += OMEGA_LP_ALPHA * (omega_raw - g_omega_lp);
		float speed = g_omega_lp * g_radius_m;

		/* accelerazione longitudinale (decel coast-down → segno negativo) */
		if (!g_primed) {
			g_speed_ms = speed;
			g_accel_lp = 0.0f;
			g_primed = true;
		} else {
			float a = (speed - g_speed_ms) / DT_S;
			g_accel_lp += ACCEL_LP_ALPHA * (a - g_accel_lp);
			g_speed_ms = speed;
		}
	}

	/* --- vibrazione: RMS della componente AC del modulo accelerometrico --- */
	if (sensor_channel_get(g_imu, SENSOR_CHAN_ACCEL_XYZ, accel) == 0) {
		float ax = (float)sensor_value_to_double(&accel[0]);
		float ay = (float)sensor_value_to_double(&accel[1]);
		float az = (float)sensor_value_to_double(&accel[2]);
		float am = sqrtf(ax * ax + ay * ay + az * az);
		g_am_mean += VIB_MEAN_ALPHA * (am - g_am_mean);
		float dev = am - g_am_mean;
		g_vib_acc += dev * dev;
		g_vib_n++;
	}

	if (sensor_channel_get(g_imu, SENSOR_CHAN_DIE_TEMP, &temp) == 0) {
		g_temp_c = (float)sensor_value_to_double(&temp);
	}

	/* --- notify STREAM a 10 Hz --- */
	if (++g_div >= STREAM_DIV) {
		g_div = 0;
		float vib_rms = (g_vib_n > 0) ? sqrtf(g_vib_acc / (float)g_vib_n) : 0.0f;
		g_vib_acc = 0.0f;
		g_vib_n = 0;

		if (wheel_ble_stream_subscribed()) {
			wheel_ble_stream_notify(g_speed_ms, g_accel_lp, g_temp_c, vib_rms);
		}
	}
}

static void sample_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	enum coast_mode last = COAST_IDLE;
	int64_t run_start = 0;

	while (1) {
		enum coast_mode mode = g_mode;

		if (mode == COAST_IDLE) {
			last = COAST_IDLE;
			k_sleep(PERIOD_IDLE);
			continue;
		}

		if (mode != last) {           /* IDLE → attivo: nuovo run */
			run_reset();
			run_start = k_uptime_get();
			last = mode;
			LOG_INF("coast-down START (mode=%d)", (int)mode);
		}

		if (g_imu && device_is_ready(g_imu)) {
			process_sample();
		}

		if (k_uptime_get() - run_start > MAX_RUN_MS) {
			LOG_WRN("coast-down timeout → IDLE");
			g_mode = COAST_IDLE;
		}

		k_sleep(PERIOD_ACTIVE);
	}
}

K_THREAD_DEFINE(sample_tid, 2048, sample_thread, NULL, NULL, NULL, 7, 0, 0);

/* ── Batteria → Battery Service (best-effort) ──────────────────────────────── */
#if IS_ENABLED(CONFIG_BT_BAS)
static void batt_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(batt_work, batt_work_fn);

static void batt_work_fn(struct k_work *work)
{
	uint8_t pct;

	if (wheel_battery_read_pct(&pct) == 0) {
		bt_bas_set_battery_level(pct);
	}
	k_work_reschedule(&batt_work, BATT_PERIOD);
}
#endif

/* ── main ──────────────────────────────────────────────────────────────────── */
int main(void)
{
	LOG_INF("AeroDrag Wheel — sensore ruota Crr (0xBB00)");

	wheel_store_init();
	wheel_store_load(&g_cfg);
	g_cfg.tire_circ_m = clampf(g_cfg.tire_circ_m, WHEEL_TIRE_CIRC_MIN, WHEEL_TIRE_CIRC_MAX);
	g_cfg.mass_kg     = clampf(g_cfg.mass_kg, WHEEL_MASS_MIN, WHEEL_MASS_MAX);
	g_radius_m = g_cfg.tire_circ_m / (2.0f * PI_F);

	g_imu = DEVICE_DT_GET(IMU_NODE);
	if (!device_is_ready(g_imu)) {
		LOG_ERR("IMU LSM6DS3TR-C non pronta — stream a zero finché non risponde");
	} else {
		imu_configure(g_imu);
		LOG_INF("IMU pronta");
	}

	wheel_battery_init();

	if (wheel_ble_init() != 0) {
		LOG_ERR("BLE init fallita");
		return 0;
	}

#if IS_ENABLED(CONFIG_BT_BAS)
	k_work_schedule(&batt_work, K_NO_WAIT);
#endif

	LOG_INF("pronto: in attesa di comando coast-down (0xBB03) dal central");
	return 0;
}
