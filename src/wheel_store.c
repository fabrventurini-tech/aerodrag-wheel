/*
 * AeroDrag Wheel — persistenza della config (tireCircM + massKg) via Zephyr
 * Settings/NVS. L'ESP32 riscrive comunque la config a ogni connessione
 * (da CONFIG 0xaa08 → 0xBB04), ma persistere evita una finestra con i default
 * dopo un riavvio e dà un comportamento da prodotto.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <string.h>

#include "wheel.h"

LOG_MODULE_REGISTER(wheel_store, LOG_LEVEL_INF);

#if IS_ENABLED(CONFIG_SETTINGS)

#include <zephyr/settings/settings.h>

#define KEY_ROOT "wheel"
#define KEY_CFG  "wheel/cfg"

static struct wheel_config s_cached;
static bool s_have;

static int wheel_settings_set(const char *name, size_t len,
			      settings_read_cb read_cb, void *cb_arg)
{
	const char *next;

	if (settings_name_steq(name, "cfg", &next) && !next) {
		if (len != sizeof(s_cached)) {
			return -EINVAL;
		}
		ssize_t rc = read_cb(cb_arg, &s_cached, sizeof(s_cached));

		if (rc >= 0) {
			s_have = true;
			return 0;
		}
		return (int)rc;
	}
	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(wheel, KEY_ROOT, NULL, wheel_settings_set,
			      NULL, NULL);

int wheel_store_init(void)
{
	return settings_subsys_init();
}

void wheel_store_load(struct wheel_config *cfg)
{
	if (s_have && cfg != NULL) {
		*cfg = s_cached;
		LOG_INF("config caricata: circ=%.3f mass=%.1f",
			(double)cfg->tire_circ_m, (double)cfg->mass_kg);
	}
}

void wheel_store_save(const struct wheel_config *cfg)
{
	if (cfg == NULL) {
		return;
	}
	s_cached = *cfg;
	s_have = true;

	int rc = settings_save_one(KEY_CFG, cfg, sizeof(*cfg));

	if (rc) {
		LOG_WRN("settings_save_one fallito (%d)", rc);
	}
}

#else /* Settings non abilitato: no-op (config solo in RAM) */

int wheel_store_init(void)
{
	return 0;
}

void wheel_store_load(struct wheel_config *cfg)
{
	ARG_UNUSED(cfg);
}

void wheel_store_save(const struct wheel_config *cfg)
{
	ARG_UNUSED(cfg);
}

#endif /* CONFIG_SETTINGS */
