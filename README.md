# Crash Detection for a DAVI M300 Laser Head on an ABB Robot

A load-cell based crash/collision detector mounted between the robot tool flange and a **DAVI M300 laser head** on a 6-axis **ABB** robot arm.

The main priority of the device is to **immediately shut off the laser and stop robot movement** when a crash or collision is detected, to prevent additional damage to the equipment and the risk of fire.

<p align="center">
  <img src="%5B05%5D%20PICTURES/Final%20assembly.jpg" width="600" alt="Final assembly: probe with load cells and electronics enclosure">
</p>

---

## How it works

- Two **20 kg bar load cells** measure the force in the **X** and **Y** directions. Forces in the **Z** direction show up as a combination of the X and Y signals because the assembly bends.
- The signals are digitized by a **24-bit ADS1256** ADC and processed by an **Arduino Nano**.
- The Nano compares each direction (**X+, X−, Y+, Y−**) against its own threshold.
- When a threshold is exceeded, the Nano drops its output signal. In the robot cabinet, an optocoupler level converter turns that into a **24 V input on the ABB controller**, which stops the robot and turns off the laser.

### Fail-safe output

| Nano output (D7) | Meaning | Robot |
|---|---|---|
| **HIGH (5 V)** | Everything OK, armed | may run |
| **LOW (0 V)** | Crash, fault or still booting | **STOP + laser off** |

The output is **active while everything is OK**. A broken cable, lost power, a frozen Nano or an ADC fault therefore also stops the robot. A crash is **latched** until the Nano is reset.

---

## Repository contents

```
[01] CODE/crash_detector/crash_detector.ino   Arduino sketch
[02] PROBE 3D MODEL/                          STEP models of the load-cell probe (DAVI base plate, load-cell base plate, laser base tube, full assembly)
[03] CASE 3D MODEL/                           STEP models of the ADS1256 / Nano enclosure and clip plate
[04] 3D PRINTED FILES/CASE PRINTED FILES.3mf  Print-ready enclosure parts
[05] PICTURES/                                Photos and render
```

---

## Hardware

| Part | Notes |
|---|---|
| 2× 20 kg bar load cell | One per axis (X, Y) |
| ADS1256 24-bit ADC module | PGA 64, input buffer on, 2000 SPS |
| Arduino Nano (ATmega328P, 5 V) | Clone with **old bootloader** (see below) |
| DST-1R4P optocoupler board (5 V → 24 V) | In the robot cabinet. Pick the output type (NPN/PNP) to match the robot's digital input. ABB inputs typically expect +24 V for "1" (PNP). |
| 24 V → 5 V buck converter | In the robot cabinet, set to 5.0 V. Powers the Nano and ADS1256. |
| 6-core cable | Load cells → ADS1256 |
| 3-core twisted cable | Enclosure → robot cabinet (5 V, GND, SIG) |

---

## Wiring

### Load cells → ADS1256 (6-core cable, shared excitation)

| Wire | Load cell X | Load cell Y | ADS1256 |
|---|---|---|---|
| E+ | red | red | 5V |
| E− | black | black | GND |
| S1+ | green | | AIN0 |
| S1− | white | | AIN1 |
| S2+ | | green | AIN2 |
| S2− | | white | AIN3 |

### ADS1256 → Arduino Nano

| ADS1256 | Nano |
|---|---|
| 5V | 5V |
| GND | GND |
| SCLK | D13 |
| DIN | D11 |
| DOUT | D12 |
| DRDY | D2 |
| CS | D10 |
| PDWN | D8 |

The ADS1256 module outputs 3.3 V logic on DOUT/DRDY, which the 5 V Nano reads as HIGH (above its 3.0 V threshold). Keep these wires short. No level shifter is used.

### Enclosure → robot cabinet (3-wire cable)

| Wire | Enclosure side | Cabinet side |
|---|---|---|
| 5V | Nano 5V | Buck converter OUT+ |
| GND | Nano GND | Buck converter OUT− and opto board input GND |
| SIG | Nano D7 | Opto board IN1 |

On the output side of the opto board: VCC → +24 V, GND → 0 V, OUT1 → ABB digital input. In the robot program, configure the input so that **0 = stop motion and laser off**.

> Do not connect USB and the external 5 V supply to the Nano at the same time.

---

## Uploading the code

1. Open `[01] CODE/crash_detector/crash_detector.ino` in the Arduino IDE. No external libraries are needed, only the built-in `SPI` and `avr/wdt`.
2. **Tools → Board:** `Arduino Nano`
3. **Tools → Processor:** `ATmega328P (Old Bootloader)`. The Nano used here has the old bootloader, so uploading fails with the default setting.
4. Select the port and upload.

**Note on the old bootloader and the watchdog:** the sketch uses the hardware watchdog (250 ms). With the old bootloader, a watchdog reset can leave the Nano stuck in a reset loop. This is safe, because the output stays at STOP, but the Nano won't recover with the reset button. **Disconnect and reconnect power** to recover. Under normal operation the watchdog never fires.

---

## Usage

### Serial monitor / plotter

Use **115200 baud**. The Serial Plotter shows `X`, `Y` and the running peaks `Xmax`, `Xmin`, `Ymax`, `Ymin`.

| Command | Action |
|---|---|
| `p` | Print peaks since reset, plus suggested thresholds (peak × 1.25) ready to paste into the code |
| `r` | Reset peaks |
| `t` | Re-tare (robot must be standing still; not possible after a crash) |

> **Opening the Serial Monitor/Plotter resets the Nano.** This clears a latched crash and re-tares, so only open it while the robot is standing still in its home pose.

### Setting the thresholds

1. Move the robot to its **home pose** and power up or reset the Nano. It tares at startup.
2. Run all robot programs a few times.
3. Send `p`. The Nano prints lines like `const float THR_X_POS = 29.3;` that already include the 25 % margin.
4. Paste them into the **USER SETTINGS** section of the sketch and upload again.

The current thresholds are **30 N** in every direction. Re-learn them whenever the tool, payload or robot programs change.

### Startup

On power-up the output holds **STOP**. The Nano initializes the ADS1256, verifies the SPI communication and tares both axes, which takes about 1 s. Then it switches the output to **OK** and prints `ARMED`. The robot must be standing still during startup, always in the same pose.

### When a crash is detected

- The output drops to STOP immediately after 3 consecutive samples over the threshold (about 5 ms).
- During the next 300 ms the Nano records the peak force, then prints a report:
  ```
  !!! CRASH  dir=X+  trigger=31.2  peakX=58.4  peakY=-6.1  -> output latched, reset Nano to re-arm
  ```
- To re-arm, check the tool and reset the Nano.

### Faults (also latch STOP)

- ADS1256 not responding or failing the register read-back check
- ADC saturation (overload or broken sensor wire)
- DRDY timeout
- Watchdog timeout

---

## Configuration (top of the sketch)

| Setting | Default | Description |
|---|---|---|
| `THR_X_POS`, `THR_X_NEG`, `THR_Y_POS`, `THR_Y_NEG` | 30.0 | Crash thresholds per direction (positive magnitudes) |
| `MARGIN` | 1.25 | Margin used for the suggested thresholds |
| `COUNTS_PER_UNIT_X/Y` | 2736.0 | ADC counts per newton. A negative value flips the axis direction. |
| `CONFIRM_SAMPLES` | 3 | Consecutive samples over the threshold before triggering |
| `CRASH_WINDOW_MS` | 300 | Time after the trigger in which the crash peak is captured |
| `OUT_OK_LEVEL` | HIGH | Output level meaning "OK" (keep HIGH for fail-safe) |

**Calibration:** values are approximately newtons at the load cell, assuming a 1.0 mV/V cell. For exact values, set the scale to `1.0`, hang a known mass in the axis direction, read the counts, and set `COUNTS_PER_UNIT = counts / (kg × 9.81)`. Threshold detection also works without exact calibration.

---

## Safety notice

This device is a collision detection aid. It is **not a certified safety function** (e.g. per ISO 13849) and does not replace the robot's emergency stop circuit, safety controller or any other safeguarding.

---

<p align="center">
  <img src="%5B05%5D%20PICTURES/Final%20probe.jpg" width="45%" alt="Probe">
  <img src="%5B05%5D%20PICTURES/Final%20case.jpg" width="45%" alt="Enclosure">
</p>
