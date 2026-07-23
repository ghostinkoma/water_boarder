# WaterBoader (PIC12F1840) — Interface Specification

Contract between the **WaterBoader** sensor (PIC12F1840, firmware `i2c_wb_5V.c` / `i2c_wb_3V3.c`) and the
host (**AquaController**, ESP32). Supersedes the PIC10F322 spec (that device was retired;
it lacked MSSP/CPS). Overview: [README.md](README.md).

- Device: **PIC12F1840** @ 16 MHz INTOSC, 5 V (3.3 V supported, see §6).
- Role: non-contact capacitive **water-level sensor** + **I2C slave**.

---

## 1. Electrical / bus

- I2C **7-bit address 0x20**, **100 kHz**, hardware MSSP with **clock stretching** (SEN=1).
- **4.7 kΩ pull-ups on the host side**; identical supply voltage host↔PIC (mixed 3.3/5 V
  causes read errors).
- Slave never actively drives SDA/SCL high (open-drain).

## 2. Register map (BME280-style register pointer)

Write `[reg]` sets the pointer; a following read returns that register (**low byte first**
for 2-byte registers); write `[reg][data…]` writes it. A read with no pointer set returns
`0x11`. The sensor free-runs, so there is no measurement handshake.

| addr | name | acc | len | meaning |
|---|---|---|---|---|
| 0x00 | ResetDevice | W | 1 | write pointer 0x00 → soft reset |
| 0x03 | SetWaterBoarderValue | R/W | 2 | water **ON** boundary (value units) |
| 0x04 | SetWaterHisWidth | R/W | 2 | hysteresis; **OFF** = boundary + width |
| 0x05 | ReadyTempValue | R | 1 | 1 = temperature ready |
| 0x06 | ReadyWaterValue | R | 1 | 1 = water value ready |
| 0x07 | GetTempValue | R | 2 | raw 10-bit die-temp ADC (0..1023) |
| 0x08 | GetWaterBoarderValue | R | 1 | **1 = water, 0 = dry** (PIC decision) |
| 0x0A | SaveConfig | W | 1 | write **0xA5** → save config to EEPROM |
| 0x0B | ConfigValid | R | 1 | 1 = valid EEPROM config loaded at boot |
| 0x0C | ResetCause | R | 1 | b0 POR b1 BOR b2 **WDT** b3 MCLR b4 soft-RESET |
| 0x10 | ReadyNCO | R | 1 | 1 = value ready |
| 0x11 | GetNCORaw | R | 2 | **windowed differential value** (default read) |
| 0x20 | Gate | R/W | 1 | gate length, ~100 µs units (1..200) |
| 0x21 | OutSel | R/W | 1 | 0=diff(prod) 1=sense 2=ref (debug) |
| 0x22 | SenseRaw | R | 2 | raw sense count |
| 0x23 | RefRaw | R | 2 | raw reference count |
| 0x24 | CalAir | R/W | 2 | calibration ref — open air |
| 0x25 | CalDry | R/W | 2 | calibration ref — dry glass |
| 0x26 | CalWater | R/W | 2 | calibration ref — water behind glass |

Values are **16-bit unsigned**. `GetNCORaw` is biased around `DIFF_BIAS = 20000`; water
**lowers** it. Address mismatch → NAK (host detects absence).

## 3. Measurement

- Differential CPS: sense = CPS3/RA4, reference = CPS0/RA0, `value = 20000 + (sense − ref)`.
- Gate 200 (~20 ms) rejects 50 Hz mains; a 16-deep sliding window drops the 4 highest and
  4 lowest and averages the middle 8 (integer, overflow-safe 32-bit sum).
- I2C is fully interrupt-driven; a read always returns the latest completed snapshot, never
  blocks on a measurement, and needs no BUSY/READY handshake.

## 4. Water decision (on the PIC)

```
if (!water) water = (value <= wbound);
else        water = (value <  wbound + hyst) ? true : false;   // OFF at wbound+hyst
```

`wbound` and `hyst` are set over I2C (0x03/0x04) or restored from EEPROM. Because sensing is
differential, the absolute `wbound` is stable enough to store and reuse.

## 5. Calibration & autonomy

1. Host runs the 3-state wizard (`WaterBoader::calibrate`): **air**, **dry glass**,
   **water**, 50 samples each → per-state **median** + **σ**.
2. `wbound = midpoint(dry, water)`, `hyst = 0.7 × σ_dry` (floor 2).
3. Host pushes gate/wbound/hyst/cals (0x20/03/04/24-26) and writes **SaveConfig (0x0A=0xA5)**.
4. The PIC stores them in data EEPROM (magic `0xA6` written last for integrity) and reloads
   them on every boot. `ConfigValid` (0x0B) reports success. **With no host present the PIC
   keeps deciding water level correctly.**

If firmware that changes the value space is flashed, the magic is bumped so old EEPROM
reads invalid → `ConfigValid = 0` → host re-calibrates.

## 6. Robustness

- **WDT ON**, ~1 s period, `CLRWDT` each loop → a hung firmware auto-resets and reloads
  EEPROM config (self-heal). **BOR ON** (voltage sag) and **STVREN ON** (stack) also reset.
- `ResetCause` (0x0C) tells the host *why* it last reset (WDT = it had hung).
- Host driver retries every I2C op `WB_RETRY` (3) times; `recover()` resets the PIC, waits
  out a possible watchdog reset, and reports the cause.

## 7. 3.3 V operation

Two firmware files, differing only in the temperature range: **`i2c_wb_5V.c`** (high range)
and **`i2c_wb_3V3.c`** (low range, `FVRCON=0xA0`; high range needs VDD >= 3.6 V so 3.3 V must
use low range). Flash the file for your voltage. On the host, match the temperature cal:
5 V → `setTempCal(4.8,4.0,94)`; 3.3 V → `setTempCal(3.3,2.0,N)` with N re-derived at a known
temperature (`N = idealADC - rawADC`, `ideal = (3.3-2*Vt)/3.3*1023`, `Vt = 0.659-(T+40)*0.00132`).
Water sensing works at either voltage — just re-calibrate the boundary at the operating
voltage (verified at 3.3 V: dry/water gap ~81, comparable to 5 V's ~88). Host and PIC must
share the SAME supply voltage.

## 8. Acceptance criteria

1. XC8 builds `i2c_wb_5V.c` and `i2c_wb_3V3.c` with no errors, well within Flash/RAM.
2. Host reads 0x20 reliably at 100 kHz under load (no hang).
3. Water toward/away from the glass moves `value` monotonically; `GetWaterBoarderValue`
   toggles at the calibrated boundary with hysteresis; host and PIC decisions agree.
4. After `SaveConfig` + power cycle, `ConfigValid = 1` and the boundary/gate persist.
5. A deliberate firmware hang is recovered by the WDT; `ResetCause` shows the WDT bit.

## 9. Known limitations

- Through-glass coupling is weak; if the dry↔water gap approaches the operating noise, the
  wizard warns (`separation < 4σ`) — improve the electrode/coupling or accept edge chatter.
- Die temperature is ~±1 °C near the cal point (part-to-part offset; one-point corrected).
- `GetNCORaw` is a relative differential value, not an absolute capacitance.
