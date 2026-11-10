Tilt Globe - Waveshare ESP32-S3-LCD-1.28 (GC9A01 round LCD + QMI8658 IMU)

  A shaded, textured planet Earth rendered in software. Tilt the board and the world spins
  in that direction (it is treated as a free ball, the planet's axis does not matter).
  The view starts centred on a random place on Earth.

  - BOOT button: short press = jump to a new random place, hold ~1 s = recalibrate "level"

  Files in this sketch folder:
    tilt_globe.ino   this file (hardware, input, main loop)
    globe_render.h   the renderer
    earth_texture.h  512x256 Earth map (RGB565)

  Library needed: "GFX Library for Arduino" (moononournation)
  Board: ESP32S3 Dev Module, USB CDC On Boot = Disabled (this board uses a CH343 USB-serial chip),
         Flash 16MB, PSRAM = QSPI PSRAM (recommended), Partition scheme: any with >= 1.5 MB app is fine
         (the default works too).
  
  Open in Arduino IDE (or your favourite IDE), compile and run
