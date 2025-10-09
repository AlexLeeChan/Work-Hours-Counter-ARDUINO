# Work Timer
# Work Hours Tracker

I needed a way to keep track of the time spent repairing amplifiers. Since I often work on several units in parallel—sometimes pausing one while waiting for parts— I decided to build a dedicated work hour counter. I had a nice 4-button keypad at stock, so I configured the system to handle four independent timers, one for each device.
Although originally designed for amplifier repairs, this timer can be used to measure and manage the working time of any activity you choose.

> 🧪 **Live simulation:** [Wokwi Project](https://wokwi.com/projects/342067577537692242)

<p align="left">
<img src="https://github.com/user-attachments/assets/b8af622c-f801-44d8-b0b6-046c725ac7e1" width="300">
<img src="https://github.com/user-attachments/assets/ea07894d-e26a-42d5-a1e0-a4023a19f929" width="300">
</p>


## Table of Contents
- [Features](#features)
- [Demo](#demo)
- [Hardware](#hardware)
- [Pinout](#pinout)
- [Firmware Overview](#firmware-overview)
- [Build & Upload](#build--upload)
- [Configuration](#configuration)
- [How to Use](#how-to-use)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Features!



- **4 independent timers** — one per button/channel  
- **Responsive LCD UI**
  - Dirty‑cache LCD updates (writes only when content changes)
  - Optional boot animation (two‑line progress bar)
- **Hold‑to‑reset with countdown**
  - Shows “Hold reset: X” after 1 s hold, resets at 3 s
  - Run/Pause toggle is blocked during countdown
- **Non‑blocking beeper**
  - Class‑based sequencer (no `delay()`)
  - Works with **active** buzzers or **passive** piezos (`tone()`)
- **Robust persistence**
  - Compact `SaveImage` with **CRC‑16/X25** integrity
  - **EEPROM ring buffer** (multi‑slot wear leveling)
  - Optional **FRAM** backend (no wear limits)
- **Efficient timekeeping**
  - Direct H:M:S increment (no division/modulo in the hot path)

## Demo

Try the full system directly in your browser using Wokwi:

👉 **Open the Wokwi Simulation:** https://wokwi.com/projects/342067577537692242

- Toggle virtual buttons to switch timers, start/stop, and reset.  
- LCD updates and buzzer feedback work as on real hardware.

## Hardware

- **Arduino** (Uno / Nano / Pro Mini or similar)
- **LCD 16×2** (HD44780 compatible, 4‑bit parallel)
- **4× push buttons** (to GND, using `INPUT_PULLUP`)
- **Beeper**
  - Active buzzer (simple on/off)
  - OR Passive piezo (uses `tone()`)
- *(Optional)* **I²C FRAM** (Adafruit I²C FRAM)  
- *(Optional)* LCD contrast control via PWM

## Pinout

| Function             | Pin                    |
|----------------------|------------------------|
| LCD RS               | D2                     |
| LCD EN               | D8                     |
| LCD D4..D7           | D4, D5, D6, D7        |
| LCD Contrast (PWM)   | D3 (PWM)               |
| Beeper               | D13                    |
| Button 1..4          | D11, D12, D9, D10      |

> You can change pins by editing the `PIN_*` constants near the top of the sketch.

## Firmware Overview

- **Debouncing** — `Deb` + `Debounce` structs provide stable press/release detection and one‑shot edge events.  
- **Beeper** — Class‑based, non‑blocking sequencer with patterns for click, start, pause, reset confirmation, and countdown tick.  
- **Persistence** — `SaveImage` (hours/minutes/seconds + sequence + CRC). `EepromStore` ring buffer across multiple slots; `FramStore` (optional).  
- **UI** — Header (“Amplifier N” / “Hold reset: X”), time `HHH:MM:SS`, dirty‑bit redraw minimizes LCD writes.  
- **Main loop** — `debounce → hold‑to‑reset → time accumulate → autosave → redraw LCD → update beeper`.

## Build & Upload

1. Open the `.ino` file in **Arduino IDE**.  
2. Select the correct board and port.  
3. Adjust feature flags if needed (see below).  
4. Click **Upload**.

Alternatively, use Arduino CLI:

```
arduino-cli compile --fqbn arduino:avr:uno .
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno .
```

## Configuration

Set these at the top of the sketch:

```
#define ENABLE_STARTUP_ANIM   1    // Show two-line boot animation
#define USE_COUNTDOWN         1    // Show "Hold reset: X" and block toggling
#define VERIFY_WRITES         1    // Verify writes by reading back
#define USE_PASSIVE_BUZZER    0    // 0=active buzzer, 1=passive piezo (tone)
#define USE_FRAM              0    // 1=Use Adafruit I2C FRAM (0x50)
#ifndef EEPROM_NUM_SLOTS
#define EEPROM_NUM_SLOTS      32   // Max number of EEPROM wear-leveling slots. The firmware automatically caps the number of slots if needed.
#endif
```

Timing constants:

```
constexpr uint8_t       DEBOUNCE_MS               = 10;      // Milliseconds for button debounce stabilization
constexpr uint8_t       CONTRAST_PWM_DUTY         = 0;       // PWM value for LCD contrast (0-255)
constexpr unsigned long SAVE_PERIOD_MS_EEPROM     = 30000UL; // 30 seconds auto-save interval for EEPROM
constexpr unsigned long SAVE_PERIOD_MS_FRAM       = 5000UL;  // 5 seconds auto-save interval for FRAM
constexpr unsigned long RESET_HOLD_MS             = 3000UL;  // 3 seconds continuous hold required for reset
constexpr unsigned long COUNTDOWN_SHOW_DELAY_MS   = 1000UL;  // 1 second hold before the countdown display starts
constexpr uint16_t      BOOT_ANIMATION_MS         = 700;    // Duration of boot animation in milliseconds
```

## How to Use

- **Select / Start / Pause**
  - Tap a button to select that channel.
  - Tap the **same** button again to toggle Run/Pause.

- **Hold‑to‑Reset**
  - Hold the **active** channel’s button:
    - After **1 s**, the LCD shows “Hold reset: 3” (then 2, 1).
    - At **3 s**, the timer resets to `00:00:00` and a confirmation tone plays.
  - Releasing during countdown **cancels** the reset and does not toggle run/pause.

Autosave occurs:
- Every `SAVE_PERIOD_MS` while running
- On **Pause**
- On **Channel switch**



- In **EEPROM mode**, the sketch uses a **ring buffer** with `EEPROM_NUM_SLOTS` slots for wear leveling. It always writes to the next slot and loads the **newest valid** slot at boot using sequence numbers and CRC.  
- In **FRAM mode** (`USE_FRAM=1`), data is stored at address `0x00` (no wear leveling needed). On startup, the code tries FRAM first and falls back to EEPROM if not found. The LCD displays “Using FRAM” or “FRAM not found / Using EEPROM”.  
- With `VERIFY_WRITES=1`, each save is read back and verified.

## Troubleshooting

| Problem                                   | Fix |
|-------------------------------------------|-----|
| No sound                                  | Check `USE_PASSIVE_BUZZER` vs hardware and wiring to `PIN_BEEPER`. |
| LCD blank / gibberish                     | Check RS/EN/D4–D7 wiring, PWM contrast, and pin defines. |
| FRAM not detected                         | Check I²C wiring, power, and address (0x50). The sketch will fallback to EEPROM. |



## License

MIT License —  Feel free to use and adapt this project for your bench, workshop, or lab.
