# JC8012P4A1C_I_W_Y – USB-Webcam-Demo (Logitech C270)

Einfache ESP-IDF-Demo: Eine USB-Webcam (UVC) am **Guition JC8012P4A1C_I_W_Y**
(ESP32-P4, 10,1" MIPI-DSI 800x1280) wird live auf dem Display angezeigt.
Getestet mit einer **Logitech C270**.

Die Demo ist eigenständig und (noch) nicht Teil von
[espVT](https://github.com/Meisterschulen-am-Ostbahnhof-Munchen/espVT); Display-Ansteuerung
und Ausrichtung (Querformat 1280x800) sind aber von dort übernommen.

## Ablauf

```
C270 --USB 2.0 HS, isochron--> usb_host_uvc (MJPEG)
     --> Hardware-JPEG-Decoder (RGB565)
     --> PPA: skalieren + 270° drehen, zentriert in den DPI-Framebuffer
     --> JD9365-Panel (2 DSI-Lanes, Double Buffering)
```

- Die Kamera wird über die Komponente
  [`espressif/usb_host_uvc`](https://components.espressif.com/components/espressif/usb_host_uvc)
  angesprochen. Die Demo probiert der Reihe nach 1280x720@30, 1280x720@10, 800x600@30,
  640x480@30 und 640x480@15 (MJPEG), bis die Kamera ein Format annimmt.
- MJPEG ohne Huffman-Tabellen ist kein Problem: Der IDF-JPEG-Treiber setzt dann die
  Standard-Tabellen ein.
- Es wird immer nur das neueste Bild angezeigt; ältere Bilder gehen sofort an den Treiber
  zurück. So gibt es keine Pufferengpässe und wenig Verzögerung.
- Beim Einstecken listet das Log alle Formate der Kamera auf, danach alle 5 s die
  Statistik (angezeigte fps, übersprungene Bilder, Puffer-Underflows).

## Hardware

| | |
|---|---|
| Board | Guition JC8012P4A1C_I_W_Y, „New Panel“ (V2/V3 auf dem Etikett), ESP32-P4 **ab Rev. v3.0** |
| Kamera | Logitech C270 (USB-ID `046d:0825`), andere UVC-Kameras mit MJPEG sollten auch gehen |
| Kamera-Anschluss | Type-C-Buchse **„High speed USB“** (USB3 im Schaltplan), USB-A-auf-C-OTG-Adapter |
| Stromversorgung / Flashen | Type-C-Buchse **USB-TTL** (USB1, CH340C), Netzteil ≥ 1 A empfohlen |

Laut Schaltplan liegen alle VBUS-Pins auf `USB5V_IN`: Die Kamera wird über die
USB-TTL-Buchse mitversorgt.

## Bauen und Flashen

Voraussetzung: **ESP-IDF 6.1**.

```sh
. ~/.espressif/tools/activate_idf_v6.1.sh      # bzw. export.sh der eigenen IDF-Installation
idf.py set-target esp32p4                      # nur beim ersten Mal
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor           # Port anpassen; Monitor beenden mit Ctrl-]
```

Die Komponenten (`esp_lcd_jd9365`, `usb`, `usb_host_uvc`) holt der IDF Component Manager
beim ersten Build; die Versionen sind in `dependencies.lock` festgehalten.

## Messwerte (C270, 1280x720 MJPEG)

```
I (1959) uvc: Allocating 4 USB transfers for ISOC. Each: 12240 bytes, 4 ISOC packets, 3060 MPS
I (1967) uvc_demo: Stream opened: 1280x720@30 MJPEG
I (2310) uvc_demo: Showing 1280x720 MJPEG
I (11366) uvc_demo: 19.5 fps displayed, 26 skipped (late), 0 underflows
I (16411) uvc_demo: 20.0 fps displayed, 36 skipped (late), 0 underflows
```

Angezeigt werden 16–20 fps. Die Grenze liegt in der Verarbeitung (Decoder, PPA und
Warten auf den Bildwechsel laufen nacheinander), nicht beim USB: Die Kamera liefert mehr,
die Differenz steht als „skipped (late)“ im Log. Vereinzelte `uvc-isoc: missed EoF`
(verlorene Isochron-Pakete) sind harmlos; das Bild wird dann verworfen.

## Anpassen

In `main/main.c`:

- **Farben vertauscht (rot/blau):** `JPEG_DEC_RGB_ELEMENT_ORDER_BGR` → `..._RGB`
- **Bild steht auf dem Kopf:** `PPA_SRM_ROTATION_ANGLE_270` → `..._90`
- **Spiegelbild (Selfie-Ansicht):** in `display_show()` `.mirror_x = true` setzen
- **Andere Auflösung:** Liste `s_formats[]`; mit z. B. 960x544 sind höhere Bildraten möglich

## Dateien

| Datei | Inhalt |
|---|---|
| `main/main.c` | Display-Init (MIPI-DSI/JD9365), UVC-Stream, JPEG-Decoder, PPA |
| `main/jc8012_lcd_init.c` | JD9365-Init-Sequenz aus dem Hersteller-BSP (über espVT) |
| `main/idf_component.yml` | Abhängigkeiten aus der ESP Component Registry |
| `sdkconfig.defaults` | ESP32-P4, PSRAM 200 MHz, 16 MB Flash, USB-Host-Einstellungen |

## Lizenz

Apache-2.0, siehe [LICENSE](LICENSE). `main/jc8012_lcd_init.c` stammt von
Espressif Systems (Apache-2.0).
