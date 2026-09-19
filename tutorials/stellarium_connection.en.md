# Connecting the mount to Stellarium (LX200 protocol)

🌐 [Русский](stellarium_connection.md) | **English**

A separate firmware is used for Stellarium: [`src/stellarium_firmware.ino`](../src/stellarium_firmware.ino).
Stellarium sees the ESP32 as a **Meade LX200 (compatible)** mount; the link is over USB (serial, 9600 8N1).
All the math (converting encoder angles to RA/Dec) and the alignment model are computed on the ESP32.
The easiest way to set the parameters is through the web interface over Wi-Fi (the ESP32 brings up its own
access point); text commands over serial also work.

## 1. Flashing

- Arduino IDE → board **ESP32 Dev Module** → open `stellarium_firmware.ino` → upload.
- Partition scheme: the default one will do — the sketch takes ~82% of it.
  If you plan further development, select **"Huge APP (3MB No OTA/1MB SPIFFS)"** ("Tools →
  Partition Scheme") — it leaves headroom. After changing the partition scheme, check on the settings page
  that the saved site slots and axis settings are still there.
- The USB port in the firmware runs at **9600 baud** (Stellarium opens it at that speed),
  so open the "Serial Monitor" at 9600 too.

## 2. Setup via the web interface (Wi-Fi)

On power-up the ESP32 brings up its own Wi-Fi access point:

- network **`redstar01`**, password **`1234567890`**;
- settings page: **https://192.168.4.1/** (self-signed certificate)

1. On your phone or laptop, connect to the `redstar01` network. Internet on that device
   will disappear (the ESP32 does not share internet) — this is normal, for the duration of the setup.
2. Open `https://192.168.4.1/` in a browser. The browser will warn once that the
   certificate is self-signed: in Chrome — "Advanced" → "Proceed", in other browsers —
   "Accept the risk"/"Continue". This is safe: the certificate is embedded in the firmware and is valid
   only in this network.
3. At the top of the page is the live state (time, site, encoder angles, RA/Dec), below are the forms.
   The page itself is in Russian; the original button and field labels are given here in «guillemets»:
   - **UTC time** («Время UTC») — a field for entering the time in UTC. The **"Take time from this
     device"** button («Взять время с этого устройства») puts the browser's current time (converted
     to UTC) into the field, after which press **"Apply"** («Применить») — only then does the time get
     into the ESP32;
   - **Location** («Место наблюдения») — **three site slots** (stored in the ESP32 memory).
     Each slot has: a name (optional), latitude and longitude (north/east `+`, south/west `-`),
     a **"Take GPS"** button («Взять GPS» — the coordinates are taken from the browser, i.e. a phone with GPS, and
     put into the slot's fields; on the first press allow location access),
     a **"Save"** button («Сохранить» — write the slot to memory) and a **"Use"** button
     («Использовать» — make the slot the current observation site). The current, i.e. active, slot
     is highlighted and marked "in use" («используется»); the "Clear" button («Очистить») resets the slot;
   - **Axes and encoder zeros** («Оси и нули энкодеров») — the "Invert AZM axis"/"Invert ALT axis"
     checkboxes («Инверсия оси AZM»/«Инверсия оси ALT» — equivalent to `AZDIR -1`/`ALTDIR -1`)
     and the `AZZERO`/`ALTZERO` zeros; the "AZ = 0 now" and "ALT = 0 now" buttons («AZ = 0 сейчас»,
     «ALT = 0 сейчас») make the current position the axis zero: press "ALT = 0 now" with the tube
     horizontal, "AZ = 0 now" with the tube pointing north (see §3 "Encoder zeros").
     Set the directions and zeros before alignment: after changing them the Sync points are wrong —
     press `CLEAR` and align again;
   - **Alignment** («Привязка») — a table of the saved Sync points (axis angles, true angles,
     residual) with a "Delete" button («Удалить») for each, a button to reset all points (`CLEAR`)
     and the text `STATUS`.

   Answers to commands are shown at the bottom of the page; everything is saved in the ESP32 memory in the same way as when entering commands over serial.

Wi-Fi and LX200 work simultaneously: Stellarium is connected over USB while the settings are being
set from the page. The device's Wi-Fi adapter connects to only one network, so while you work with the
page the laptop will have no internet (setting things up from a phone is more convenient).

If the enabled Wi-Fi is not needed, it can be turned off: at the beginning of `stellarium_firmware.ino`
replace `#define WEB_UI 1` with `#define WEB_UI 0` and reflash the board.

## 3. Time and observation site (serial commands)

If the web interface is unavailable, the same parameters are set with text commands.
Commands are entered as text terminated by a newline in the "Serial Monitor" (USB, 9600). While Stellarium
is using the port, you cannot enter commands into it — enter commands before connecting
Stellarium (the Wi-Fi page works at any time).

| Command | Example | Purpose |
|---|---|---|
| `T YYYY-MM-DD HH:MM:SS` | `T 2026-09-17 18:00:00` | **UTC** time |
| `SLOT <n> <lat> <lon> [name]` | `SLOT 1 55.0583 73.2950 Dacha` | save a site slot (n = 1..3) |
| `SLOTUSE <n>` | `SLOTUSE 1` | make the slot the current site |
| `SLOTCLEAR <n>` | `SLOTCLEAR 1` | clear the slot |
| `AZDIR 1` / `AZDIR -1` | `AZDIR -1` | azimuth axis direction |
| `ALTDIR 1` / `ALTDIR -1` | `ALTDIR 1` | altitude axis direction |
| `AZZERO <deg>` | `AZZERO -340` | azimuth encoder zero offset |
| `ALTZERO <deg>` | `ALTZERO -340` | altitude encoder zero offset |
| `STATUS` | `STATUS` | current state (including the alignment point list) |
| `PTDEL <n>` | `PTDEL 2` | delete alignment point number n (numbers as in `STATUS`) |
| `CLEAR` | `CLEAR` | reset alignment (Sync points) |
| `HELP` | `HELP` | command list |

The site is stored in three slots (`SLOT`/`SLOTUSE`/`SLOTCLEAR` commands or the buttons on the
web page), and the axis settings — all of this is saved in the ESP32 memory and persists after
a reboot: after power-up the active slot immediately becomes the current site.
The time must be set after every power-up (there is no RTC on the board; if the `T` command is not
set, the firmware compile time is used as an approximation).

Example of initial setup (enter one line at a time):

```
T 2026-09-17 18:00:00
SLOT 1 55.0583 73.2950 Dacha
SLOTUSE 1
STATUS
```

The latitude and longitude must match the location set in Stellarium
(settings window → "Location"), and the time must be UTC.

### Checking the axis directions

`STATUS` (and the "State" table, «Состояние», on the web page) shows three numbers for each axis.
Using azimuth as an example (the firmware prints in Russian; the translation is in brackets):

```
Энкодер AZ: raw 7456 (163.80°), после настройки: 163.800°
[AZ encoder: raw 7456 (163.80°), after setup: 163.800°]
```

- `raw 7456` — the "raw" encoder steps, 0…16383 per full revolution
  (1 step = 360/16384 ≈ 0.022° ≈ 1.3 arcminutes);
- `(163.80°)` — the same angle in degrees, still without direction and zero:
  `steps × 360 / 16384`;
- `после настройки: 163.800°` (after setup) — the axis angle in the mount's frame after `AZDIR`/`AZZERO`:
  `norm(direction × raw° + zero)`, where azimuth is normalized to 0…360° and altitude to −180…+180°.

This is still **not** the real azimuth or the real tube altitude — the real angles are shown by the
next line (`Углы монтировки` in `STATUS`, "Mount", «Монтировка», on the web page), already taking
alignment into account.

Rotate the mount:

- in azimuth **clockwise** (viewed from above) — the "after setup" azimuth should increase;
- tube **up** — the "after setup" altitude should increase.

If it increases the other way, set `AZDIR -1` and/or `ALTDIR -1`.

### Encoder zeros (AZZERO/ALTZERO)

The encoder is an absolute sensor within one revolution: it always outputs 0…16383 steps,
but where the "zero" of this revolution is depends only on how the magnet was installed
relative to the axis during assembly, and has nothing to do with the mount's mechanics. The
`AZZERO`/`ALTZERO` zeros shift the encoder scale so that its zero is in a convenient place; the values are stored
in the ESP32 memory.

**In what tube position to set the zeros:**

| Axis | Tube position | Action | Why |
|---|---|---|---|
| Altitude (ALT) | tube horizontal, looking at the horizon (check with a level) | "ALT = 0 now" button (or `ALTZERO`) | the tube's travel then lies around 0° and stays entirely in one half of the scale |
| Azimuth (AZ) | tube pointing north (e.g. at Polaris) | "AZ = 0 now" button (or `AZZERO`) | "after setup" is measured from north (convenient to read, does not affect accuracy) |

The buttons take axis inversion (`AZDIR`/`ALTDIR`) into account automatically. If you set the commands
manually, the sign depends on the direction: with `ALTDIR 1` it is `ALTZERO -<raw°>`, with
`ALTDIR -1` it is `ALTZERO +<raw°>` (the same for azimuth).

**Why this matters for altitude but hardly for azimuth.** A circular scale cannot be turned
into a number without a discontinuity: for azimuth it is at 0°/360°, for altitude at ±180°
(see "Checking the axis directions").

- For azimuth the discontinuity is harmless: azimuth is a circle, and the firmware always compares angles along the
  short arc. `AZZERO` is mostly for convenience.
- For altitude the discontinuity is dangerous: if it falls inside the tube's real travel, "after setup"
  jumps by 360° in the middle of the movement (e.g. `+179.9°` → `−179.9°`). Sync points on
  opposite sides of the discontinuity are seen by the model as ~360° apart, and the crosshair
  jumps or drifts away. Zeroing with the tube horizontal moves the travel 180° away from the discontinuity.

Usually the zeros can be left alone: the very first alignment (Sync) subtracts a constant offset.
Checking is easy: slowly sweep the tube through its entire real travel and watch the "after
setup" altitude — if there is no jump through ±180° anywhere, everything is fine.

Order: first `AZDIR`/`ALTDIR` and the zeros, then Sync. After changing a direction or a
zero, the old Sync points become invalid (they store the previous axis readings) —
do `CLEAR` and align again.

If the tube goes past vertical (crossing the zenith or nadir), the firmware reports the
coordinates folded: azimuth +180°, altitude 180°−altitude. This is the same point in the sky,
so the crosshair keeps moving instead of freezing at the zenith.

### Time and site accuracy

Time and site enter the calculations in the same way — only through sidereal time
`LST = GMST(UTC) + longitude`. If the LST in the ESP32 and Stellarium differ, the crosshair
is rotated around the celestial axis by that angle: **there is no error at Polaris**,
and toward the celestial equator it grows and reaches its maximum (outwardly it looks like "the crosshair
does not match the azimuth grid"). 1 second of time = 15 arcseconds of sky,
1 minute = 15 arcminutes, 1 hour = 15°; 1 arcminute of longitude equals 4 seconds of
time. A latitude error gives an offset of the same order as the error itself.

| Time discrepancy | Sky rotation | Error at the equator | Error at δ = 45° |
|---|---|---|---|
| 1 s | 15″ | 15″ | 11″ |
| 10 s | 2.5′ | 2.5′ | 1.8′ |
| 30 s | 7.5′ | 7.5′ | 5.3′ |
| 1 min | 15′ | 15′ | 10.6′ |
| 5 min | 1.25° | 1.25° | 53′ |
| 1 hour | 15° | 15° | 10.6° |

For visual observing, keep the pointing error at most ~1/3 of the eyepiece field of view.
Hence the permissible time discrepancy: a wide-field eyepiece (~2° field) — up to 2–3 minutes,
25 mm (~1.4°) — up to 1.5–2 minutes, 10 mm (~0.6°) — up to 40 seconds. A practical guideline is
**no more than 10 seconds**: that is an error of ≤2.5′ at the equator (none at Polaris) and less than one
encoder step (14 bits ≈ 1.3′); setting it more precisely is pointless.

- **Time** is easiest to take with the "Take time from this device" button (the phone's
  or laptop's clock is NTP-synchronized), then "Apply". If entering manually —
  UTC only: a timezone mistake is 15° per hour.
- The clock of the computer running Stellarium must also be synchronized (Linux:
  `timedatectl status` should show synchronization). The ESP32's and the
  computer's own drift over a night is fractions of a second and can be neglected.
- If the time in Stellarium is shifted manually or sped up, the ESP32 does not know about it:
  set the same moment with the `T` command (with accelerated time there will be no match in principle).
- **Longitude**: phone GPS (±10 m) has a margin of hundreds of times; even a 0.1° error
  (a few kilometers) gives ≤6′. Watch only the sign: east `+`, west `-` — a flipped
  sign turns the crosshair by tens of degrees.
- **Latitude**: an error gives roughly the same offset in altitude; GPS also has a margin here,
  and a map accurate to ~10 km gives ≤5′. Do not swap latitude and longitude.

Setup order: first the time (`T`), the active site slot (`SLOTUSE`) and the axis settings
(`AZDIR`/`ALTDIR`, zeros — see below), then Sync. Alignment points are computed from the current
time, site and axis readings: if you fix them after alignment, the model becomes
invalid — do `CLEAR` and align again.

## 4. USB connection

The port is detected automatically:

- **Windows**: after installing the CP210x or CH340 driver (depending on the board),
  `COMx` will appear in "Device Manager".
- **Linux**: `/dev/ttyUSB0` (or `/dev/ttyACM0`). The user must be in the `dialout` group
  (`sudo usermod -aG dialout $USER`, then log in again).
- **Linux, important**: the name `/dev/ttyUSBx` is not persistent — after replugging the board or a USB
  glitch the number can change (`ttyUSB0` → `ttyUSB1`), and Stellarium, having remembered the old path,
  stops connecting with the `cannot open serial device` error. Specify the stable path
  from `/dev/serial/by-id/` (it does not depend on enumeration order):
  ```
  ls /dev/serial/by-id/
  # usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
  ```
- **Stellarium from snap** (`stellarium-daily`, the snap name may differ): you need to allow USB-serial access once:
  ```
  sudo snap connect stellarium-daily:raw-usb
  ```

### Configuring the telescope in Stellarium

1. `F2` → **"Telescope Control"** plugin → "Configure"
   (or `Ctrl+1` — the "Telescopes" window).
2. "Add a new telescope".
3. "Telescope controlled by:" → **"Stellarium, directly (through serial port or network)"**.
4. "Device model:" → **LX200 (compatible)**.
5. "Connection:" → **"Serial port"** (not "Network (TCP)").
6. "Serial port:" → `COMx` / `/dev/ttyUSB0` (Linux: better a path from
   `/dev/serial/by-id/`, see §4).
7. "Equinox" → **J2000 (default)** — the firmware reports J2000 coordinates.
8. "Connection delay" — 0.5 s by default.
9. Save and press **"Connect"**.

After connecting, the telescope crosshair appears in the sky. Until any Sync has been made,
the crosshair sits at the celestial pole (next to Polaris) and moves together with the
mount's knobs: the firmware assumes that at power-up the tube was pointing at the pole.
If the mount is leveled and the tube was pointing at Polaris at power-up, you can
even point without alignment — the first Sync replaces this rough model with an exact one.

## 5. Star alignment

While there is no Sync point, the firmware assumes that at power-up (or after `CLEAR`)
the tube was pointing at the celestial pole: the crosshair sits next to Polaris and follows the mount.
The first Sync replaces this rough model. A convenient order: point the tube at Polaris and
make the first Sync on it, then refine the alignment with 1–2 stars in another part of the sky.

1. Make sure the time (UTC) and site in the ESP32 match the Stellarium settings
   (see §3 "Time and site accuracy").
2. Point the mount at a bright star and center it in the eyepiece.
   - Do the alignment while the tube is **below the zenith** (not folded over the top): the correction
     model measures altitude from the normal axis position, and a Sync on the fold will shift
     it for the whole sky.
3. Select that star in Stellarium (click on the star).
4. Press **`Ctrl+Shift+1`** — "Sync telescope #1 with the selected object"
   (in the "Slew" window these are the "Current object" → "Sync" buttons).
5. Repeat steps 2–4 for 2–3 stars spread across the sky (you can do more; up to
   6 points are stored).
   - 1 star — constant axis offsets;
   - 2 stars — the same offsets, averaged over the two points;
   - 3 or more stars — a linear error model (least squares over all stored points).
6. To point at an object, select it in Stellarium and press `Ctrl+1` — the crosshair
   shows the target, and you bring the tube to it manually by the difference in positions.
7. You can reset the alignment (e.g. after disassembling the mount) with the `CLEAR`
   command over serial or the button on the web page. After the reset the crosshair counts
   from the pole again, and the current tube position is taken as the pole.

A repeated Sync near an already aligned star (within ~5° in axis angles)
**updates** its point instead of adding an almost identical second one: if after pointing
the crosshair did not match the star, just point more precisely and press Sync again — this is
safe. When 6 points have been collected, a new Sync elsewhere replaces the oldest one.
`STATUS` and the "Alignment" table on the web page show the point list and the
residual (model error at each point); there you can delete a point with a button, or over
serial with the `PTDEL <number>` command.

Tip: after each Sync, check the accuracy on the same star — the crosshair
should coincide with the star. If it has drifted away, press Sync again.

## 6. Common problems

| Symptom | Cause/solution |
|---|---|
| Stellarium has no port in the list | Linux: no permissions (`dialout`), snap without `snap connect ...raw-usb`, a cable without data lines (charge-only) |
| Port busy / will not connect | The "Serial Monitor" is open (close it) or another program is using the port |
| After "Disconnect" it will not reconnect, the log shows `cannot open serial device` | The board re-enumerated under a different name (`/dev/ttyUSB0` → `/dev/ttyUSB1`) — specify a port from `/dev/serial/by-id/...` (see §4) |
| The page `https://192.168.4.1/` does not open | The device is not connected to the `redstar01` network (Wi-Fi is looking at another network); check that this exact firmware is flashed |
| The browser says "Connection not secure" / complains about the certificate | This is by design: the certificate is self-signed and valid only in this network. Click "Advanced" → "Proceed" (Chrome) or "Accept the risk" (other browsers) |
| No GPS coordinate request | Allow location access for the browser in the phone settings; on a computer without a GPS module the coordinates may be determined by Wi-Fi/network — that is inaccurate |
| The sketch will not compile: "Sketch too big" | The selected partition scheme is too small: take the default one or "Huge APP (3MB No OTA/1MB SPIFFS)" (see §1) |
| The crosshair twitches and "flies away" | No site slot selected (`SLOTUSE`) or no time set (`T`), or wrong axis directions (`AZDIR`/`ALTDIR`), or large alignment errors (see §3) |
| The crosshair jumps/drifts when the tube passes a certain place: `STATUS` shows the "after setup" altitude jumping ≈+180° → −180° | The altitude scale discontinuity fell inside the tube's travel: make the tube horizontal, press "ALT = 0 now" (or set `ALTZERO`), then `CLEAR` and Sync again (see §3 "Encoder zeros") |
| Alignment "broke" after changing `AZDIR`/`ALTDIR`/`AZZERO`/`ALTZERO` | The Sync points store the previous axis readings: do `CLEAR` and align again |
| The crosshair does not move | Wrong port selected; the SkySafari firmware is flashed (different protocol); Stellarium is set to "Network (TCP)" instead of "Serial port" |
| Large misses | UTC time not set (`T`), site mismatch (active slot coordinates), wrong `AZDIR`/`ALTDIR`, too few alignment stars or they are close to each other |
| The crosshair is rotated around Polaris: it matches near the pole but is far off toward the equator | Sidereal time mismatch: inaccurate time (`T`) or longitude (active slot); 1 minute of time = 15′ at the equator (see §3). If fixed after alignment — `CLEAR` and Sync again |
| The crosshair is offset in altitude by almost the same amount all over the sky | Wrong latitude (active slot) or alignment errors |
| Alignment lost after power-off | By design: Sync points are stored in RAM only; site slots and axis settings persist |

## 7. Notes

- Stellarium does not send either the time or the site coordinates to the mount — they must be set
  on the web page or with commands (`T`, `SLOT`/`SLOTUSE`).
- The web interface lives over Wi-Fi only: the ESP32's USB port is an ordinary serial port,
  a browser cannot open a page over it.
- While there are no alignment points, coordinates are counted from the celestial pole (see §5); this is not a "bug"
  and requires no setup — the first Sync switches the firmware to the normal model.
- GPS coordinates are taken from the device on which the page is open (a phone with GPS) and
  put into the slot fields (saving is done with the `SLOT` command): browsers provide geolocation
  only over HTTPS, which is why the page is served over HTTPS with a self-signed certificate.
  The device answers `http://192.168.4.1/` with a redirect to HTTPS.
- The firmware does not support an LX200 TCP connection (host:port) — use "Serial port"
  (USB).
- Refraction is not modeled; for push-to accuracy this is insignificant.
