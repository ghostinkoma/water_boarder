# WaterBoader (PIC12F1840)

Non-contact **capacitive water-level sensor** as an **I2C slave** (default `0x20`).
A PIC12F1840 senses water through a tank wall (glass/plastic) with a differential
electrode pair, decides *water present / absent*, and reports it — plus the raw value
and its own die temperature — to a host (the **AquaController**, ESP32) over I2C.

It calibrates on-site via an interactive wizard and **persists the calibration in its
own EEPROM**, so it keeps working correctly even with no host attached (redundancy).

---

## Why it works

* **Differential sensing** — `value = sense_electrode − reference_electrode`. Common-mode
  drift (temperature, humidity, VDD, most EMI) hits both electrodes and cancels; only real
  water near the *sense* electrode moves the value. This is what makes an **absolute,
  fixed threshold** stable enough to store and reuse (unlike an adaptive baseline, which
  would track a slow water change and miss it).
* **Water lowers the value.** Boundary is set between the dry and water levels; the PIC
  applies hysteresis so the output does not chatter.
* **Robustness** — hardware I2C (MSSP) handled entirely in an interrupt so a read never
  collides with a measurement; **watchdog** (auto-reset on hang), **brown-out reset**,
  **stack-overflow reset**, and a readable **reset-cause** register.

## Hardware

| PIC pin | use |
|---|---|
| 1 VDD | 5 V (or 3.3 V — see temperature note) |
| 2 RA5 | activity LED |
| 3 RA4 | **CPS3 sense electrode** (route away from I2C; ICSP-independent) |
| 4 RA3 | MCLR / VPP (ICSP) |
| 5 RA2 | **SDA** |
| 6 RA1 | **SCL** (also ICSPCLK — pull PICkit after programming) |
| 7 RA0 | **CPS0 reference electrode** (also ICSPDAT — jumper off for programming) |
| 8 VSS | GND |

* **4.7 kΩ pull-ups** on SDA/SCL to the host VDD; **same supply voltage** on host and PIC.
* CPS needs a real board (not a breadboard) — stray capacitance swamps the pF-level signal.

## Firmware (PIC)

`i2c_wb_5V.c` (5 V) and `i2c_wb_3V3.c` (3.3 V) — two paste-ready builds that differ ONLY in
the temperature range (`TEMP_HIGH_RANGE`; 3.3 V needs low range). Edit `i2c_wb_5V.c` and run
`gen_3v3.sh` to regenerate the 3.3 V file. Build with XC8, `-mcpu=PIC12F1840`. Free-running
`main()` measures; the SSP interrupt answers I2C from a memory-resident snapshot. Config
words: `WDTE=ON`, `BOREN=ON`, `STVREN=ON`, `MCLRE=OFF`, `FOSC=INTOSC`.
Match the host temperature cal to the build: 5 V → `setTempCal(4.8,4.0,94)`; 3.3 V →
`setTempCal(3.3,2.0,N)` (re-derive N). Water sensing is re-calibrated at the operating voltage.

## Library (host / AquaController)

Drop the `WaterBoader/` folder into your Arduino/PlatformIO library path.

```cpp
#include <Wire.h>
#include <WaterBoader.h>

WaterBoader wb(0x20);

void setup() {
  Wire.begin();
  wb.begin();                 // pass true on classic AVR to enable A4/A5 pull-ups
}

void loop() {
  int w = wb.water();         // 1 = water, 0 = dry, -1 = I2C error
  if (w == 1) { /* alarm / pump logic */ }
  float t = wb.tempC();       // die temperature
  delay(500);
}
```

### One-time on-site calibration

```cpp
WbConfig cfg;
if (wb.calibrate(cfg)) {      // interactive: air -> dry glass -> water, 50 samples each
  wb.pushConfig(cfg);
  wb.save();                  // persist to the PIC's EEPROM -> autonomous thereafter
}
```

On the next boot `wb.configValid() == 1` and the PIC restores the calibration by itself.

### Fault recovery

```cpp
if (wb.value() < 0) {         // persistent I2C failure?
  wb.recover();               // reset the PIC, wait out a possible watchdog reset,
}                             // then report resetCause() (WDT / soft-RESET / BOR ...)
```

See `examples/WaterBoaderDemo` for a sketch that uses **every** method.

## Temperature (secondary feature)

Die temperature via the internal indicator (AN1333). The absolute reading needs a
**one-point calibration** (large part-to-part offset in high range). Defaults are for
**5 V** (`VDD=4.8, mode=4, offset=94`). For **3.3 V**: build the PIC with
`TEMP_HIGH_RANGE=0` and call `wb.setTempCal(3.3, 2.0, offset)` with a re-derived offset.
It is ~±1 °C near the calibration point — fine for over-temperature warnings, not lab-grade.

## Files

```
i2c_wb_5V.c / i2c_wb_3V3.c       PIC firmware (5V / 3.3V; gen_3v3.sh regenerates 3V3)
WaterBoader/
  WaterBoader.h / .cpp           host driver (this library)
  examples/WaterBoaderDemo/…     full-API example
  library.properties
  README.md  SPEC.md
```

See **SPEC.md** for the exact register/behaviour contract.
