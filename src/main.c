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
 *   omega    = |gyro|  = √(gx²+gy²+gz²)      [rad/s]  (modulo: tilt-invariante)
 *   radius   = tireCircM / (2·pi)            [m]
 *   speedMs  = omega · radius                [m/s]    (v_center della finestra)
 *   accelMs2 = pendenza LS della finestra    [m/s²]   (decel coast-down = NEGATIVA)
 *   vibRMS   = RMS( |accel| - media_mobile ) [m/s²]   (rugosità superficie)
 *   tempC    = temperatura die IMU           [°C]
 *
 * speedMs/accelMs2 vengono da una REGRESSIONE LINEARE su finestra (v_center +
 * pendenza, riferiti allo stesso istante): elimina il bias da doppio low-pass
 * che sovrastimava il Crr. Vedi i commenti a ACCEL_WIN_N e a process_sample().
 *
 * CRITICO: il giroscopio va portato a FONDO-SCALA ±2000 dps (≈34.9 rad/s). Col
 * default ±245 dps saturerebbe a ~3 km/h e la misura sarebbe inutilizzabile.
 * A ±2000 dps il tetto è ~42 km/h su ruota 700c (omega·r): i coast-down partono
 * tipicamente ≤35 km/h; oltre il fondo-scala i campioni saturi vengono scartati.
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

/* Vibrazione (rugosità superficie): media mobile del modulo |accel| */
#define VIB_MEAN_ALPHA   0.05f                    /* media mobile |accel|       */

/* Accelerazione longitudinale via REGRESSIONE LINEARE a finestra.
 * Sostituisce la vecchia cascata omega-LP → differenza finita → accel-LP, che
 * introduceva un lag differenziale fra speed e accel → bias del Crr (sovrastima
 * crescente con la velocità). La pendenza LS su una finestra uniforme è a
 * "fase-zero" al centro: v_center (media) e a_center (pendenza) sono riferiti
 * allo STESSO istante → coppia (v,a) coerente. 25 campioni @100 Hz = 250 ms:
 * media via il rumore gyro, la curvatura di a(v) sull'intervallo è trascurabile.
 * Tarare su HW se serve. */
#define ACCEL_WIN_N      25
#define ACCEL_WIN_MIN    8                        /* campioni minimi per stimare */
#define SPIN_LOG_RADS    5.0f                     /* soglia log asse dominante  */

/* IMU full-scale / ODR */
#define IMU_ODR_HZ       208
/* Fondo-scala gyro ±2000 dps, passato a sensor_attr_set come sensor_value
 * {val1, val2} = 34.906585 rad/s (sono i due campi di sensor_value, NON versioni). */
#define GYRO_FS_VAL1     34
#define GYRO_FS_VAL2     906585
#define GYRO_FS_RADS     34.906585f               /* ±2000 dps in rad/s         */
#define GYRO_SAT_RADS    (0.98f * GYRO_FS_RADS)   /* soglia clip/flat-top gyro  */
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
static volatile bool g_cfg_dirty;     /* config da persistere a fine run (vedi set_config) */

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

	/* La PERSISTENZA su flash la fa SEMPRE il sample_thread (unico writer NVS →
	 * niente race su settings_save_one da due thread). Qui marchiamo solo "dirty".
	 * Durante un run il nuovo raggio vale dal run SUCCESSIVO: process_sample usa
	 * g_run_radius (snapshot a START), così una CONFIG ricevuta a metà corsa non
	 * altera il raggio in corsa (niente salto speed→accel → fit Crr protetto). */
	g_cfg_dirty = true;
	LOG_INF("config: circ=%.3f m mass=%.1f kg (r=%.4f m)%s",
		(double)g_cfg.tire_circ_m, (double)g_cfg.mass_kg, (double)g_radius_m,
		(g_mode == COAST_IDLE) ? "" : " [vale dal prossimo run]");
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
	v.val1 = GYRO_FS_VAL1; v.val2 = GYRO_FS_VAL2;
	rc = sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
	if (rc) {
		LOG_ERR("set gyro FS ±2000dps rc=%d — VERIFICARE: il default ±245dps "
			"satura a ~3 km/h!", rc);
	} else {
		LOG_INF("IMU: ODR=%d Hz, gyro ±2000 dps, accel ±16 g", IMU_ODR_HZ);
	}
}

/* ── Campionamento + calcolo + notify ──────────────────────────────────────── */

/* Stato run (resettato a ogni transizione IDLE→attivo).
 * Ring buffer della velocità grezza (omega·raggio) per la regressione: i campioni
 * sono equispaziati a DT_S, requisito della forma chiusa della pendenza LS. */
static float    g_speed_buf[ACCEL_WIN_N];   /* ring: speed grezza (omega·raggio) */
static bool     g_sat_buf[ACCEL_WIN_N];     /* ring: campione con gyro in clip    */
static uint8_t  g_buf_head;                 /* prossima posizione di scrittura    */
static uint8_t  g_buf_count;                /* campioni nel ring (≤ ACCEL_WIN_N)  */
static uint8_t  g_sat_in_win;               /* saturi attualmente in finestra     */
static float    g_run_radius;               /* raggio snapshot a START (no jump)  */
static float    g_am_mean;
static float    g_vib_acc;
static uint16_t g_vib_n;
static bool     g_vib_primed;
static float    g_temp_c = 20.0f;
static uint8_t  g_div;
static bool     g_spin_logged;
static uint32_t g_sat_log_n;                /* rate-limit log saturazione         */

static void run_reset(void)
{
	g_buf_head = 0; g_buf_count = 0; g_sat_in_win = 0;
	g_am_mean = 0; g_vib_acc = 0; g_vib_n = 0; g_vib_primed = false;
	g_div = 0; g_spin_logged = false; g_sat_log_n = 0;
	/* snapshot del raggio: una CONFIG (0xBB04) ricevuta a run attivo NON deve
	 * alterare il raggio usato in corsa (eviterebbe un salto di speed→accel). */
	g_run_radius = g_radius_m;
}

/* Media e pendenza LS di una finestra di campioni equispaziati (passo DT_S):
 *   v_center = media(speed)                                   [m/s]
 *   a_center = Σ(k-kc)·s_k / (DT_S·Σ(k-kc)²),  kc=(n-1)/2      [m/s²]
 * La retta LS passa per (t_center, media): i due valori sono allo STESSO istante
 * (centro finestra) → nessun lag differenziale fra v e a (no bias del Crr). */
static void regress_window(float *v_center, float *a_center)
{
	int n = g_buf_count;
	int start = (g_buf_head + ACCEL_WIN_N - n) % ACCEL_WIN_N;   /* più vecchio */
	float kc = (float)(n - 1) * 0.5f;
	float sum = 0.0f, num = 0.0f, den = 0.0f;

	for (int k = 0; k < n; k++) {
		float s = g_speed_buf[(start + k) % ACCEL_WIN_N];
		float dk = (float)k - kc;
		sum += s;
		num += dk * s;
		den += dk * dk;
	}
	*v_center = sum / (float)n;
	*a_center = (den > 0.0f) ? (num / (den * DT_S)) : 0.0f;
}

static void process_sample(void)
{
	struct sensor_value gyro[3], accel[3], temp;

	if (sensor_sample_fetch(g_imu) != 0) {
		return;
	}

	/* --- velocità angolare: MODULO del vettore gyro (tilt-invariante) ---
	 * Un solo asse |gyro[asse]| sottostima un mount obliquo di cos(tilt);
	 * il modulo √(gx²+gy²+gz²) recupera tutte le componenti di rotazione. */
	if (sensor_channel_get(g_imu, SENSOR_CHAN_GYRO_XYZ, gyro) == 0) {
		float gx = (float)sensor_value_to_double(&gyro[0]);
		float gy = (float)sensor_value_to_double(&gyro[1]);
		float gz = (float)sensor_value_to_double(&gyro[2]);

		/* saturazione: un asse a ~fondo-scala è flat-top → campione inaffidabile
		 * (un coast-down avviato troppo veloce darebbe decel ~0 = garbage). */
		bool sat = (fabsf(gx) >= GYRO_SAT_RADS) ||
			   (fabsf(gy) >= GYRO_SAT_RADS) ||
			   (fabsf(gz) >= GYRO_SAT_RADS);

		float omega = sqrtf(gx * gx + gy * gy + gz * gz);
		float speed = omega * g_run_radius;

		/* diagnostica: asse di spin dominante (solo log, non entra nel valore) */
		if (!g_spin_logged && omega > SPIN_LOG_RADS) {
			int mx = (fabsf(gx) >= fabsf(gy))
				 ? ((fabsf(gx) >= fabsf(gz)) ? 0 : 2)
				 : ((fabsf(gy) >= fabsf(gz)) ? 1 : 2);
			LOG_INF("spin rilevato: asse dominante %c (omega=%.1f rad/s)",
				"XYZ"[mx], (double)omega);
			g_spin_logged = true;
		}

		/* push nel ring (mantenuto uniforme nel tempo per la pendenza LS) */
		if (g_buf_count == ACCEL_WIN_N && g_sat_buf[g_buf_head]) {
			g_sat_in_win--;             /* esce un campione saturo */
		}
		g_speed_buf[g_buf_head] = speed;
		g_sat_buf[g_buf_head] = sat;
		if (sat) {
			g_sat_in_win++;
		}
		g_buf_head = (g_buf_head + 1) % ACCEL_WIN_N;
		if (g_buf_count < ACCEL_WIN_N) {
			g_buf_count++;
		}
	}

	/* --- vibrazione: RMS della componente AC del modulo accelerometrico --- */
	if (sensor_channel_get(g_imu, SENSOR_CHAN_ACCEL_XYZ, accel) == 0) {
		float ax = (float)sensor_value_to_double(&accel[0]);
		float ay = (float)sensor_value_to_double(&accel[1]);
		float az = (float)sensor_value_to_double(&accel[2]);
		float am = sqrtf(ax * ax + ay * ay + az * az);

		if (!g_vib_primed) {
			g_am_mean = am;             /* prime: no picco gonfio al warm-up */
			g_vib_primed = true;
		} else {
			g_am_mean += VIB_MEAN_ALPHA * (am - g_am_mean);
		}
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

		/* Finestra contaminata da saturazione → NON trasmettere: una coppia
		 * (v,a) corrotta entrerebbe nel fit Crr dell'app. Il wire 16B ratificato
		 * non ha un campo "stato"; sopprimere il frame è il modo conforme di far
		 * scartare il dato all'app. (Lo stato esplicito wheel→app è la seam #32,
		 * non ancora ratificata.) */
		if (g_sat_in_win > 0) {
			if ((g_sat_log_n++ % 10) == 0) {
				LOG_WRN("gyro saturo (>±2000 dps): frame soppresso "
					"(coast-down troppo veloce)");
			}
			return;
		}
		if (g_buf_count < ACCEL_WIN_MIN) {
			return;                     /* ancora pochi campioni per stimare */
		}

		float v_center, a_center;

		regress_window(&v_center, &a_center);

		if (wheel_ble_stream_subscribed()) {
			wheel_ble_stream_notify(v_center, a_center, g_temp_c, vib_rms);
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
			/* unico writer NVS: persiste la config "dirty" a riposo (entro
			 * ~200ms; durante un run resta dirty e si salva al ritorno IDLE). */
			if (g_cfg_dirty) {
				g_cfg_dirty = false;
				wheel_store_save(&g_cfg);
				LOG_INF("config persistita");
			}
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
