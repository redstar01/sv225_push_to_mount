# Svbony SV225 Alt-Azimuth Telescope Mount Push-To Retrofit

🌐 [Русский](README.md) | **English**

DIY project: an ordinary manual alt-azimuth mount is turned into a Push-To system with star alignment. An ESP32 with two MT6701 magnetic encoders reports the axis positions to **Stellarium** (USB, Meade LX200 protocol) or **SkySafari** (Bluetooth / Wi-Fi).

<p align="center">
    <img height="500" src="./photos/1000004954.jpg"/>
</p>

## 🆕 What's new: the Stellarium version (beta)

The project is moving toward **Stellarium** — this version is already the priority and is working (beta). The SkySafari firmwares remain stable and supported.

- **USB connection** — no Bluetooth, no pairing, no phone: Stellarium sees the ESP32 as a Meade LX200 mount.
- **All the astronomy is computed on the ESP32** — sidereal time, J2000 precession, axis-angle ↔ RA/Dec conversion.
- **Setup from a phone over Wi-Fi** — the page `https://192.168.4.1/`: UTC time with one button, three observation-site slots with GPS, axis calibration, live status table.
- **Star alignment** (Sync from Stellarium) — up to 6 points, a least-squares linear model from 3 points; a repeated Sync refines a point, and residuals and deletion are available in the web table.
- **Start from the pole** — before the first alignment the crosshair sits next to Polaris and follows the mount.
- **Zenith crossing** — coordinates do not "freeze": the firmware folds the pointing direction and the crosshair keeps moving.

<p align="center">
    <img width="360" src="./photos/webui_main.jpg"/>
    <br/>
    <em>Web interface: state, time, site slots and alignment — all from a phone</em>
</p>
<!-- SCREENSHOT: put a screenshot of the main web interface screen into ./photos/webui_main.png -->

## 🚀 Project overview

### What is Push-To?
It is a system that shows the observer which way to turn the mount's knobs (after alignment to one or more reference stars) in order to point at any celestial object.
It makes finding faint galaxies, nebulae and star clusters that are invisible to the naked eye much easier.
It also helps beginner astronomers a lot when searching for objects.

### Difference from Go-To
In a Go-To system the mount points itself at celestial objects using motorized drives — that is the next level of automation and pointing convenience.
With Push-To we align to the sky and then move from one target to another by turning the mount by hand, matching the pointing data in the planetarium program.

### How does it work?
Rotation-angle sensors (encoders) are installed on the mount's axes.
An ESP32 microcontroller reads them and sends the data to a planetarium program, which sees the axis position changes and shows where the tube is pointing.
For Stellarium the conversion of angles to J2000 coordinates and the alignment model are computed right on the ESP32.

## 📎 Features

### Stellarium — beta, the priority direction
- Meade LX200 protocol, USB connection (9600 8N1) — no Bluetooth or pairing.
- J2000 coordinates; all the math on the ESP32: GMST/LST, precession (IAU 1976), alt-az ↔ RA/Dec.
- Sync alignment: 1 point — constant offsets, 2 — averaging, 3+ — linear model (least squares), up to 6 points total.
- Wi-Fi web interface: UTC time, 3 site slots with names and GPS, axis calibration (inversion, zeros), alignment point list with residuals.
- "From the pole" start without alignment; zenith/nadir crossing does not break pointing.
- No internet or phone app needed: a USB cable and a PC with Stellarium.

### SkySafari — stable versions
- Basic Encoder System protocol.
- Bluetooth connection (convenient because Wi-Fi stays free for internet).
- Wi-Fi connection (optional, requires flashing a separate firmware).

## 💻 Software

The project is written in C++/Arduino for the ESP32 platform. The source code is [here](./src).

| Firmware | Planetarium | Connection | Status |
|---|---|---|---|
| [stellarium_firmware.ino](./src/stellarium_firmware.ino) | Stellarium | USB (LX200) + Wi-Fi web setup | beta, priority |
| [bluetooth_firmware.ino](./src/bluetooth_firmware.ino) | SkySafari | Bluetooth | stable |
| [wifi_firmware.ino](./src/wifi_firmware.ino) | SkySafari | Wi-Fi | stable |

Guides: [Stellarium connection](./tutorials/stellarium_connection.en.md) · [SkySafari connection](./tutorials/skysafari_connection.en.md)

## 🎯 How to use

### Stellarium
Detailed guide — [tutorials/stellarium_connection.en.md](./tutorials/stellarium_connection.en.md).

1. Flash [stellarium_firmware.ino](./src/stellarium_firmware.ino): board **ESP32 Dev Module**, partition scheme — the default one (or "Huge APP (3MB No OTA/1MB SPIFFS)" for headroom).
2. Connect the ESP32 to the PC over USB. In Stellarium: **Telescope Control** plugin → "LX200 (compatible)" → "Serial port" → equinox **J2000**.
3. Set the time and location: Wi-Fi **`redstar01`** (password `1234567890`) → `https://192.168.4.1/` (the browser will warn once about the self-signed certificate). Button "Take time from this device" → "Apply"; in the site slot — "Take GPS" → "Use".
4. Point the mount at a star, select it in Stellarium and press **`Ctrl+Shift+1`** (Sync). Repeat for 2–3 stars spread across the sky.
5. Turn the knobs — the crosshair moves across the sky. Pick an object and bring the tube to it by hand.

Before the first alignment the crosshair sits at the celestial pole (next to Polaris) and follows the mount: the first Sync replaces this rough model with an exact one.
The time and coordinates must match Stellarium: 1 minute of time error is 15′ of miss at the celestial equator.
If the web interface is unavailable, the same parameters can be set with serial commands: `T 2026-09-17 18:00:00`, `SLOT 1 55.0583 73.2950 Dacha`, `SLOTUSE 1` (`HELP` — command list).

### SkySafari
Detailed guide — [tutorials/skysafari_connection.en.md](./tutorials/skysafari_connection.en.md).

1. Connect power via USB (for a devkit board) to the ESP32.
2. Flash `bluetooth_firmware.ino` (or `wifi_firmware.ino`) and connect to the ESP32's Bluetooth (Wi-Fi) from the device with SkySafari.
3. [In SkySafari, connect to the pre-configured telescope.](./tutorials/skysafari_connection.en.md) (encoder step count `-16383`).
4. Align to reference stars, preferably 3 or more, though 1 star will work acceptably. (If the telescope's field of view is not large, the target will always land in the eyepiece.)
5. Turn the mount's knobs and watch the telescope direction move in SkySafari. Then pick any object in the app and bring the telescope to the target.

## ⚙️ Components and materials
- ESP32 DevKit microcontroller (1 pc)
- MT6701 absolute magnetic encoders (2 pcs)
- Larger magnets for the encoders (optional, will improve accuracy) (2 pcs)
- USB-C ports (4 pcs)
- Extended imperial bolts, size 1/4-20, length 1-1/8" (6 pcs)
- Needle roller bearing 6*19 (2 pcs)
- CIATIM-F grease
- 3D-printed parts

## 🔧 Assembly
Detailed assembly instructions [here](./tutorials/assembly.en.md)

## 🖨️ 3D models

All printable models can be found [here](./models/STL)

## 📷 Photos

Photos of the assembly process and the final result can be found [here](./photos)

## 📞 Contact the author

GitHub: https://github.com/redstar01/sv225_push_to_mount

Telegram: https://t.me/redstar01 (Pavel Krasnoperov)

## Disclaimer
All information is provided "as is", without any warranties. The author is not responsible for any damage, equipment damage, injuries or other problems resulting from the use of this information. You act at your own risk.

## ❤️️ P.S. If you like the project, give it a star on GitHub!
