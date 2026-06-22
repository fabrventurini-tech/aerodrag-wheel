# AeroDrag Wheel — firmware sensore ruota Crr

Firmware del **sensore ruota** per la calibrazione del **Crr** (Coefficient of
Rolling Resistance) tramite **coast-down**. È un dispositivo BLE che espone il
servizio proprietario **`0xBB00`** consumato dall'ESP32 (BLE central), che ne
relaya lo stream all'app e ne calcola il Crr.

- **Board:** Seeed **XIAO BLE Sense** (nRF52840) · **IMU onboard:** **LSM6DS3TR-C** (I2C)
- **Alimentazione:** **LiPo** (caricatore onboard sul XIAO)
- **Toolchain:** nRF Connect SDK / Zephyr (BLE peripheral)
- **Ruolo (contract v0.3.0 §2.G):** sensore che misura **velocità** (da giroscopio)
  e fornisce i dati per il fit del Crr durante il coast-down.

> **Fonte di verità delle interfacce:** il **contratto** in
> `aerodrag-firmware/docs/CONTRACT.md` (§2, confine *firmware↔wheel*). Questo repo è
> l'implementazione del lato sensore: non modifica il contratto, lo implementa.

---

## Contratto BLE — servizio `0xBB00`

UUID base SIG `0000bbXX-0000-1000-8000-00805f9b34fb`.

| CHR | UUID | Flags | Bytes | Payload (little-endian) |
|-----|------|-------|-------|--------------------------|
| STREAM | `0xBB01` | NOTIFY 10 Hz | 16 | `float32 speedMs, accelMs2, tempC, vibRMS` |
| RESULT | `0xBB02` | NOTIFY | 6 | `float32 crr + uint8 quality + uint8 runIdx` — **legacy, non usato** (il Crr lo calcola l'app) |
| CMD | `0xBB03` | WRITE | 1 | `uint8`: `0x01` indoor · `0x02` outdoor-A · `0x03` outdoor-B · `0xFF` cancel |
| CONFIG | `0xBB04` | READ+WRITE | 8 | `float32 tireCircM + float32 massKg` |

Inoltre è esposto il **Battery Service** standard `0x180F` (addizione
non-breaking, utile lato prodotto; il central può ignorarla).

### Flusso operativo (allineato ad app + ESP32)
1. L'ESP32 si connette, scrive **CONFIG `0xBB04`** (circonferenza + massa, dai
   parametri dell'app `CONFIG 0xaa08`) e si **sottoscrive a STREAM `0xBB01`**.
2. L'app avvia un coast-down → l'ESP32 inoltra il comando su **CMD `0xBB03`**.
3. Il sensore entra in coast-down e **trasmette STREAM a 10 Hz**; l'ESP32 lo
   relaya all'app su `0xaa0c`.
4. L'app esegue il **fit dei minimi quadrati** `-a = c1 + c2·v²`
   (`aerodrag-new/src/physics/crr.ts`) e ricava il Crr; richiede `accelMs2`
   **negativa** in decelerazione, `speedMs > soglia`, ≥15 campioni.
5. `0xFF` (cancel) o fine sessione → il sensore torna **IDLE** (non trasmette).

Lo stream è **gated sul coast-down**: in IDLE il sensore non notifica (risparmio
batteria/airtime), coerente con l'uso "durante la calibrazione" del contratto.

---

## Modello fisico (montaggio sul MOZZO)

La board è sul **mozzo**: l'asse di rotazione della ruota coincide con un asse del
giroscopio. La velocità si ricava dalla **velocità angolare**:

```
omega    = |gyro[asse_spin]|            [rad/s]   (asse AUTO-rilevato)
radius   = tireCircM / (2·pi)           [m]
speedMs  = omega · radius               [m/s]
accelMs2 = LP( d(speedMs)/dt )          [m/s²]    (decel coast-down = NEGATIVA)
vibRMS   = RMS( |accel| − media_mobile )[m/s²]    (rugosità superficie)
tempC    = temperatura die IMU          [°C]
```

- **Auto-rilevamento dell'asse di spin:** il firmware individua a runtime l'asse
  giroscopico dominante (quello con |rate| sostenuto > ~5 rad/s) → il montaggio
  X/Y/Z non richiede ricompilazioni.
- `accelMs2` è **con segno** (negativa = decelera), come si aspetta il fit dell'app.

### ⚠️ Fondo-scala giroscopio — il punto critico
A 40 km/h su ruota 700c servono **~33 rad/s (~1900 dps)**. Il default del
LSM6DS3TR-C è **±245 dps (~4,3 rad/s)** → **saturerebbe a ~3 km/h**. Il firmware
imposta quindi a runtime il **fondo-scala ±2000 dps** (e ±16 g sull'accel, ODR
208 Hz). Tetto teorico a ±2000 dps ≈ **42 km/h**: i coast-down partono in genere
≤35 km/h, quindi è adeguato. *(Senza questa impostazione il sensore è inutilizzabile:
era il difetto principale dello scheletro v0.2.1.)*

---

## Build & flash (nRF Connect SDK)

Workspace-app (manifest `west.yml`):
```sh
west init -l .                 # usa questo repo come manifest
west update                    # scarica NCS/Zephyr (pin in west.yml: v2.7.0)
west build -b xiao_ble/nrf52840/sense
west flash                     # XIAO: doppio reset → bootloader UF2, oppure J-Link
```
Se hai già un workspace NCS, copia/clona questo repo come applicazione e lancia
direttamente il `west build` con la board sopra.

---

## Stato e bring-up

**🟢 Implementato** (oltre lo scheletro v0.2.1):
- Servizio GATT `0xBB00` completo (STREAM/RESULT/CMD/CONFIG) + advertising + **re-advertising
  automatico alla disconnessione** + **bonding** (anti cross-talk, si lega all'ESP32).
- IMU via **Zephyr Sensor API** (driver-agnostica) con **ODR + fondo-scala ±2000 dps/±16 g**.
- Calcolo `speedMs/accelMs2/tempC/vibRMS`, **auto-rilevamento asse di spin**, filtri
  (omega ~8 Hz, accel ~2 Hz), stream NOTIFY 10 Hz (campionamento 100 Hz).
- **CONFIG persistita** (Zephyr Settings/NVS) + clamp ai range del contratto.
- **Batteria LiPo** → Battery Service `0x180F` (lettura ADC periodica, best-effort).
- **Risparmio energetico**: campionamento ridotto in IDLE; auto-timeout di sicurezza.

**⚠️ Da verificare in bring-up sul prototipo** (non compilabile/flashabile in questo
ambiente — manca la toolchain nRF Connect SDK):
- **IMU/DTS:** che la board `xiao_ble/nrf52840/sense` esponga l'IMU col compatible/Kconfig
  giusto per la tua NCS (qui `CONFIG_LSM6DSL`); se la DTS usa un altro compatible o un
  alias `imu`, adegua `prj.conf`/overlay.
- **Asse di spin & segni:** confermare l'auto-rilevamento e che `accelMs2` sia negativa
  in decelerazione sul tuo montaggio.
- **Batteria:** pin (`AIN7`/P0.31, enable P0.14 attivo basso) e **rapporto partitore**
  (`VBAT_DIVIDER` in `wheel_battery.c`, default 2.96) sulla tua revisione di XIAO.
- **Autonomia** col LiPo: valutare deep-sleep con wake-on-motion (INT IMU) come passo
  successivo se serve più autonomia.
- **Tetto di velocità** (~42 km/h a ±2000 dps): sufficiente per il coast-down; se
  servisse di più, il sensore HW non offre FS superiore.

---

## Struttura

```
west.yml                              # manifest NCS (app standalone)
CMakeLists.txt · prj.conf · VERSION
boards/xiao_ble_nrf52840_sense.overlay# ADC batteria + enable gpio
src/
  wheel.h            # interfacce condivise + config/cmd/range
  main.c             # IMU (ODR/FS), campionamento, calcolo, stato coast-down, batteria
  wheel_ble.c        # GATT 0xBB00, advertising, connessione, BAS
  wheel_battery.c    # lettura batteria LiPo (ADC, best-effort)
  wheel_store.c      # persistenza config (Settings/NVS)
```

## Governance
Nessuna modifica al servizio `0xBB00` in autonomia: si propone nella seam
`firmware↔wheel`, si concorda, poi il capofila ratifica nel contratto
(`aerodrag-firmware/docs/CONTRACT.md`). `RESULT 0xBB02` resta legacy (il Crr lo
calcola l'app).
