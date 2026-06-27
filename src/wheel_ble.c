/*
 * AeroDrag Wheel — layer BLE: servizio 0xBB00, advertising, gestione connessione.
 *
 *   0xBB01 STREAM  NOTIFY 10 Hz  16 B: float speedMs, accelMs2, tempC, vibRMS (LE)
 *   0xBB02 RESULT  NOTIFY         6 B: legacy, non usato (il Crr lo calcola l'app)
 *   0xBB03 CMD     WRITE          1 B: 0x01 indoor / 0x02 out-A / 0x03 out-B / 0xFF cancel
 *   0xBB04 CONFIG  READ+WRITE     8 B: float tireCircM + float massKg
 *
 * Espone anche il Battery Service standard (0x180F) — addizione non-breaking,
 * utile lato prodotto; il central può ignorarla.
 */
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <string.h>

#include "wheel.h"

LOG_MODULE_REGISTER(wheel_ble, LOG_LEVEL_INF);

#define DEVNAME "AeroDrag Wheel"

/* UUID 16-bit del servizio/caratteristiche (base SIG) */
#define BT_UUID_WHEEL_SVC     BT_UUID_DECLARE_16(0xBB00)
#define BT_UUID_WHEEL_STREAM  BT_UUID_DECLARE_16(0xBB01)
#define BT_UUID_WHEEL_RESULT  BT_UUID_DECLARE_16(0xBB02)
#define BT_UUID_WHEEL_CMD     BT_UUID_DECLARE_16(0xBB03)
#define BT_UUID_WHEEL_CONFIG  BT_UUID_DECLARE_16(0xBB04)

static volatile bool s_stream_subscribed;
static int s_conn_count;          /* connessioni attive (gestione adv su bond_deleted) */

/* ── Handler GATT ──────────────────────────────────────────────────────────── */

static ssize_t cfg_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	struct wheel_config cfg;

	wheel_core_get_config(&cfg);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &cfg, sizeof(cfg));
}

static ssize_t cfg_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	struct wheel_config cfg;

	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0 || len != sizeof(cfg)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	memcpy(&cfg, buf, sizeof(cfg));
	wheel_core_set_config(&cfg);   /* il core clampa + persiste */
	return len;
}

static ssize_t cmd_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0 || len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (wheel_core_apply_cmd(((const uint8_t *)buf)[0]) != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	return len;
}

static void stream_ccc_cfg(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	s_stream_subscribed = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("STREAM notify %s", s_stream_subscribed ? "ON" : "OFF");
}

/* Tabella attributi del servizio 0xBB00. Indici:
 *   [0]=svc [1]=STREAM(decl) [2]=STREAM(val) [3]=STREAM CCC
 *   [4]=RESULT(decl) [5]=RESULT(val) [6]=RESULT CCC
 *   [7]=CMD(decl) [8]=CMD(val)
 *   [9]=CONFIG(decl) [10]=CONFIG(val)
 */
BT_GATT_SERVICE_DEFINE(wheel_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_WHEEL_SVC),

	/* (v0.3.4 §2.G) Sicurezza confine G: sottoscrizioni/scritture sensibili gated
	 * dietro cifratura (LE SM1 L≥2). GUARDRAIL: Just Works/NoInputNoOutput su
	 * entrambi i lati → Level 2 *unauthenticated* → si usa _ENCRYPT, NON _AUTHEN
	 * (con Just Works _AUTHEN non sarebbe mai soddisfatto → write rifiutate per
	 * sempre). Il wire/payload resta invariato. */
	BT_GATT_CHARACTERISTIC(BT_UUID_WHEEL_STREAM, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(stream_ccc_cfg, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT),

	BT_GATT_CHARACTERISTIC(BT_UUID_WHEEL_RESULT, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT),

	BT_GATT_CHARACTERISTIC(BT_UUID_WHEEL_CMD, BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE_ENCRYPT, NULL, cmd_write, NULL),

	BT_GATT_CHARACTERISTIC(BT_UUID_WHEEL_CONFIG,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT,
			       cfg_read, cfg_write, NULL),
);

/* L'attributo "valore" di STREAM è alla posizione [2]. */
#define ATTR_STREAM_VAL (&wheel_svc.attrs[2])

bool wheel_ble_stream_subscribed(void)
{
	return s_stream_subscribed;
}

void wheel_ble_stream_notify(float speed_ms, float accel_ms2,
			     float temp_c, float vib_rms)
{
	/* 16 B little-endian (il nRF52840 è LE → il float[] è già nel layout giusto) */
	float frame[4] = { speed_ms, accel_ms2, temp_c, vib_rms };

	(void)bt_gatt_notify(NULL, ATTR_STREAM_VAL, frame, sizeof(frame));
}

/* ── Advertising + connessione + bonding (v0.3.4 §2.G) ─────────────────────── */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x00, 0xBB),   /* 0xBB00, LE */
};
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVNAME, sizeof(DEVNAME) - 1),
};

/* Ripopola l'accept-list dai bond persistiti (callback di bt_foreach_bond). */
static void al_add_bond(const struct bt_bond_info *info, void *user_data)
{
	int *n = user_data;
	char s[BT_ADDR_LE_STR_LEN];

	if (bt_le_filter_accept_list_add(&info->addr) == 0) {
		bt_addr_le_to_str(&info->addr, s, sizeof(s));
		LOG_INF("accept-list += %s", s);
		(*n)++;
	}
}

/* Ricostruisce l'accept-list dai bond. Ritorna true se ne esiste ≥1. */
static bool rebuild_accept_list(void)
{
	int n = 0;

	bt_le_filter_accept_list_clear();
	bt_foreach_bond(BT_ID_DEFAULT, al_add_bond, &n);
	return n > 0;
}

static void advertising_start(void)
{
	/* GUARDRAIL #1 (il più pericoloso): filtrare con l'accept-list SOLO se esiste
	 * già un bond. Pubblicizzare con FILTER_CONN e lista VUOTA = nessuno può
	 * connettersi → device non accoppiabile PER SEMPRE. Senza bond → adv APERTO
	 * (bootstrap pairing); col bond ESP32 → adv filtrato verso quel solo peer. */
	/* preset connectable+scan. NB: BT_LE_ADV_CONN è deprecato da NCS 2.8 → al
	 * prossimo bump dell'SDK sostituire con BT_LE_ADV_CONN_FAST_2. */
	struct bt_le_adv_param param = *BT_LE_ADV_CONN;
	bool have_bond;
	int err;

	/* Stop preventivo: bt_le_adv_start NON ri-applica i parametri se l'adv è già
	 * attivo (ritorna -EALREADY) → per passare aperto↔filtrato bisogna ripartire. */
	(void)bt_le_adv_stop();

	have_bond = rebuild_accept_list();
	if (have_bond) {
		param.options |= BT_LE_ADV_OPT_FILTER_CONN |
				 BT_LE_ADV_OPT_FILTER_SCAN_REQ;
	}

	err = bt_le_adv_start(&param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EALREADY) {
		LOG_ERR("adv start failed (%d)", err);
		return;
	}
	LOG_INF("advertising (svc 0xBB00, name '%s') — %s", DEVNAME,
		have_bond ? "accept-list (solo bond ESP32)" : "aperto (pairing)");
}

static void on_connected(struct bt_conn *conn, uint8_t err)
{
	int rc;

	if (err) {
		LOG_WRN("connection failed (0x%02x)", err);
		advertising_start();   /* riprova ad annunciarsi */
		return;
	}
	s_conn_count++;
	LOG_INF("central connesso → richiedo cifratura (L2)");
	/* GUARDRAIL #2: L2 (unauthenticated), NON L3+ — Just Works non soddisfa L3+
	 * → le write 0xBB03/04 verrebbero rifiutate per sempre. */
	rc = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (rc) {
		LOG_WRN("bt_conn_set_security(L2) rc=%d", rc);
	}
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	if (s_conn_count > 0) {
		s_conn_count--;
	}
	LOG_INF("central disconnesso (0x%02x) → riavvio advertising", reason);
	s_stream_subscribed = false;
	advertising_start();
}

static void on_security_changed(struct bt_conn *conn, bt_security_t level,
				enum bt_security_err err)
{
	ARG_UNUSED(conn);
	if (err) {
		LOG_WRN("cifratura FALLITA (level %d, err %d) — scritture 0xBB03/04 "
			"resteranno rifiutate finché il link non è cifrato", level, err);
	} else {
		LOG_INF("link cifrato attivo: security level %d", level);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = on_connected,
	.disconnected = on_disconnected,
	.security_changed = on_security_changed,
};

/* Il bond ora esiste → il prossimo advertising_start (post-disconnect) applicherà
 * l'accept-list verso il solo ESP32. */
static void on_pairing_complete(struct bt_conn *conn, bool bonded)
{
	ARG_UNUSED(conn);
	LOG_INF("pairing complete (bonded=%d): enforcement cifratura confine G attivo",
		bonded);
}

/* GUARDRAIL #3: su bond cancellato si DEVE tornare ad adv aperto, altrimenti si
 * resterebbe filtrati verso un bond inesistente → non accoppiabile. Se siamo
 * connessi ci pensa on_disconnected; se siamo da soli con adv filtrato attivo,
 * riconciliamo subito (advertising_start fa stop+rebuild → lista vuota → aperto). */
static void on_bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
	ARG_UNUSED(id); ARG_UNUSED(peer);
	LOG_INF("bond cancellato → riconcilio advertising (re-pairing)");
	if (s_conn_count == 0) {
		advertising_start();
	}
}

static struct bt_conn_auth_info_cb auth_info_cb = {
	.pairing_complete = on_pairing_complete,
	.bond_deleted = on_bond_deleted,
};

int wheel_ble_init(void)
{
	int err = bt_enable(NULL);

	if (err) {
		LOG_ERR("bt_enable failed (%d)", err);
		return err;
	}

	/* Carica i bond persistiti PRIMA dell'advertising: se il bond ESP32 c'è già,
	 * si parte subito con l'accept-list (niente finestra aperta superflua). */
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	(void)bt_conn_auth_info_cb_register(&auth_info_cb);

	advertising_start();
	return 0;
}
