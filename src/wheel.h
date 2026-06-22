/*
 * AeroDrag Wheel — interfacce condivise fra i moduli del firmware.
 *
 * Firmware del sensore ruota Crr (coast-down). Espone il servizio BLE 0xBB00
 * (contract firmware↔wheel, §2.G). Consumer = ESP32 (BLE central), che relaya
 * lo stream all'app (0xaa0c), inoltra i comandi coast-down (da 0xaa0d → 0xBB03)
 * e scrive la config (circonferenza+massa) sul 0xBB04 (da CONFIG 0xaa08).
 *
 * Board: Seeed XIAO BLE Sense (nRF52840) — IMU onboard LSM6DS3TR-C + LiPo.
 */
#ifndef AERODRAG_WHEEL_H_
#define AERODRAG_WHEEL_H_

#include <stdbool.h>
#include <stdint.h>

/* ── Config (scritta dall'ESP32 via 0xBB04: float tireCircM + float massKg) ──
 * I range rispecchiano CONFIG 0xaa08 del firmware ESP32 (contract §2):
 *   massKg ∈ [33,200], wheelCircM/tireCircM ∈ [1.0,2.5].
 */
struct wheel_config {
	float tire_circ_m;   /* circonferenza ruota [m]  (== wheelCircM dell'app) */
	float mass_kg;       /* massa atleta+bici [kg]                            */
} __attribute__((packed));

#define WHEEL_TIRE_CIRC_DEFAULT 2.105f   /* 700x25c                          */
#define WHEEL_MASS_DEFAULT      78.0f
#define WHEEL_TIRE_CIRC_MIN     1.0f
#define WHEEL_TIRE_CIRC_MAX     2.5f
#define WHEEL_MASS_MIN          33.0f
#define WHEEL_MASS_MAX          200.0f

/* ── Comandi coast-down (0xBB03, 1 byte) — identici a WHEEL_CMD dell'app ──── */
enum coast_cmd {
	COAST_CMD_INDOOR  = 0x01,
	COAST_CMD_OUT_A   = 0x02,
	COAST_CMD_OUT_B   = 0x03,
	COAST_CMD_CANCEL  = 0xFF,
};

/* Stato coast-down interno */
enum coast_mode {
	COAST_IDLE = 0,
	COAST_INDOOR,
	COAST_OUT_A,
	COAST_OUT_B,
};

/* ── Core (main.c) — invocate dal layer BLE sulle scritture GATT ──────────── */

/* 0xBB03 write: applica un comando coast-down. Ritorna 0 se valido,
 * -EINVAL se il byte non è un comando noto (→ ATT error lato BLE). */
int  wheel_core_apply_cmd(uint8_t cmd);

/* 0xBB04 write: aggiorna la config (clampata ai range) e la persiste. */
void wheel_core_set_config(const struct wheel_config *cfg);

/* 0xBB04 read: copia la config corrente. */
void wheel_core_get_config(struct wheel_config *out);

/* ── BLE (wheel_ble.c) ────────────────────────────────────────────────────── */
int  wheel_ble_init(void);

/* True se un central è sottoscritto alle NOTIFY di STREAM (0xBB01). */
bool wheel_ble_stream_subscribed(void);

/* Invia una NOTIFY STREAM (0xBB01): 16 B LE = speedMs, accelMs2, tempC, vibRMS. */
void wheel_ble_stream_notify(float speed_ms, float accel_ms2,
			     float temp_c, float vib_rms);

/* ── Batteria (wheel_battery.c) ───────────────────────────────────────────── */
int  wheel_battery_init(void);
/* Legge la percentuale batteria [0..100]. Ritorna 0 ok, <0 se ADC non pronto. */
int  wheel_battery_read_pct(uint8_t *pct_out);

/* ── Persistenza config (wheel_store.c, Zephyr Settings/NVS) ──────────────── */
int  wheel_store_init(void);
/* Carica la config persistita in *cfg (se assente, lascia *cfg invariata). */
void wheel_store_load(struct wheel_config *cfg);
void wheel_store_save(const struct wheel_config *cfg);

#endif /* AERODRAG_WHEEL_H_ */
