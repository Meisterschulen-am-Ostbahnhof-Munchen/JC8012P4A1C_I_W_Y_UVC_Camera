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
  angesprochen. Die Demo nimmt die Formatliste, die die Kamera beim Einstecken meldet, und
  wählt das größte MJPEG-Format bis 1280x720. Lehnt die Kamera ab, kommt das nächstkleinere
  Format dran; größere Formate bis 1280x960 nur als letzte Wahl.
- Die Bildrate wird ebenfalls aus den Angaben der Kamera gewählt: die höchste angebotene
  Rate bis `MAX_FPS` (30). Bietet die Kamera nur höhere an, die niedrigste davon; bei
  stufenlos einstellbaren Kameras wird auf 30 begrenzt. Fest eingestellt ist nur die
  Obergrenze, nicht die Rate selbst. Das Log zeigt pro Format die angebotenen Raten.
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
| Kamera | Logitech C270 (USB-ID `046d:0825`), andere UVC-Kameras mit MJPEG sollten auch gehen (siehe unten) |
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

## Getestete Kameras

| Kamera | Übertragung | gewähltes Format | angezeigt |
|---|---|---|---|
| Logitech C270 | isochron | 1280x720 MJPEG | 16–20 fps |
| UGREEN HDMI-zu-USB-Grabber mit AHD-Kamera (720p25) | Bulk | 1280x720 MJPEG @ 30 (angeboten 60/50/30) | 20 fps |
| UGREEN HDMI-zu-USB-Grabber mit PAL-Kamera über PAL-zu-HDMI-Wandler | Bulk | 1280x720 MJPEG @ 30 | 20 fps |
| USB-Kameramodul mit ESP32, Platine „ESP32-USB-14A“ | – | 1280x720 MJPEG @ 15 | 14,6 fps (volle Kamerarate) |

Das ESP32-Kameramodul bietet MJPEG 1280x720@15, 800x480@20, 640x480@25 und 480x320@25 an.
Selten kommt von ihm ein fehlerhaftes JPEG; der Rahmen wird übersprungen.

## AHD-Kamera über HDMI-Grabber

Dieselbe Firmware zeigt auch eine analoge **AHD-Rückfahrkamera** an. Die Kette:

```
AHD-Kamera (720p25) → AHD-zu-HDMI-Wandler → UGREEN HDMI-zu-USB-Grabber → JC8012 (USB-HS)
```

Der Grabber meldet sich als UVC-Gerät (MJPEG bis 1920x1080) und streamt per **Bulk** in
Nutzlastpaketen zu 16 KB. Angezeigt werden 1280x720 mit 20 fps.

- **Korrigierter UVC-Treiber nötig:** `usb_host_uvc` 2.5.1 (und auch 2.6.0) entfernt bei
  Bulk-Streams nur den ersten und letzten UVC-Header eines Bildes; die Header dazwischen
  landen im JPEG, und der Hardware-Decoder meldet bei jedem Bild „marker parsed by the
  decoder is not supported by the hardware“. Das Projekt nutzt deshalb eine korrigierte Kopie
  in `components/usb_host_uvc` (siehe `README.espVT.md` dort). Gemeldet bei Espressif:
  [esp-usb#651](https://github.com/espressif/esp-usb/issues/651).
- **„Video Format Not Supported“ / „视频格式不支持“** auf dem Display kommt vom
  AHD-zu-HDMI-Wandler: Er hat das Kameraformat nicht erkannt. Einmal aus- und wieder
  einstecken genügte; danach blendet der Wandler kurz „AHD 720p@25“ ein. Analoge Kameras
  gibt es als CVBS, AHD 720p/1080p, TVI und CVI; Kamera und Wandler müssen dasselbe Format
  sprechen.
- Ohne Signal am HDMI-Eingang zeigt der Grabber sein UGREEN-Logo.

## Anpassen

In `main/main.c`:

- **Farben vertauscht (rot/blau):** `JPEG_DEC_RGB_ELEMENT_ORDER_BGR` → `..._RGB`
- **Bild steht auf dem Kopf:** `PPA_SRM_ROTATION_ANGLE_270` → `..._90`
- **Spiegelbild (Selfie-Ansicht):** in `display_show()` `.mirror_x = true` setzen
- **Andere Auflösung:** `PREFERRED_MAX_W/H` (Standard 1280x720); kleinere Formate wie
  960x544 erlauben höhere Bildraten

## Dateien

| Datei | Inhalt |
|---|---|
| `main/main.c` | Display-Init (MIPI-DSI/JD9365), UVC-Stream, JPEG-Decoder, PPA |
| `main/jc8012_lcd_init.c` | JD9365-Init-Sequenz aus dem Hersteller-BSP (über espVT) |
| `main/idf_component.yml` | Abhängigkeiten aus der ESP Component Registry |
| `components/usb_host_uvc/` | `espressif/usb_host_uvc` 2.5.1 mit korrigiertem Bulk-Parser |
| `sdkconfig.defaults` | ESP32-P4, PSRAM 200 MHz, 16 MB Flash, USB-Host-Einstellungen |

## Lizenz

Apache-2.0, siehe [LICENSE](LICENSE). `main/jc8012_lcd_init.c` und
`components/usb_host_uvc/` stammen von Espressif Systems (Apache-2.0).
