/*
 * WaterBoaderDemo.ino - exercises every WaterBoader method.
 *
 * Startup wizard:
 *   - shows the PIC reset cause and whether it booted with valid EEPROM config
 *   - C = run the calibration wizard, push + save to the PIC EEPROM
 *   - S / 10s timeout = measure with the config the PIC already holds
 * Then it streams live readings and auto-recovers on persistent I2C errors.
 *
 * Wiring: PIC12F1840 running i2c_wb_5V.c (or i2c_wb_3V3.c), I2C 0x20, 4.7k pull-ups on SDA/SCL,
 * same supply voltage on both sides.  On classic AVR pass true to begin() to also
 * enable the internal A4/A5 pull-ups.
 */
#include <Wire.h>
#include <WaterBoader.h>

WaterBoader wb(0x20);
WbConfig    cfg;
static uint8_t miss = 0;
static uint32_t tsec = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  wb.begin(/*avrPullups=*/true);
  delay(300);

  Serial.println(F("=== WaterBoader demo ==="));

  int rc = wb.resetCause();                      // 0x0C
  if (rc >= 0) wb.printResetCause((uint8_t)rc);

  int valid = wb.configValid();                  // 0x0B
  bool haveCfg = (valid == 1);
  Serial.println(haveCfg ? F("stored calibration: YES (PIC EEPROM)")
                         : F("stored calibration: none"));
  Serial.println(F("C = calibrate / S = start.  (10s timeout = start)"));

  // simple 10s prompt
  bool doCal = !haveCfg;
  unsigned long t0 = millis();
  while (millis() - t0 < 10000) {
    if (Serial.available()) {
      char c = tolower(Serial.read());
      if (c == 'c') { doCal = true;  break; }
      if (c == 's') { doCal = false; break; }
    }
  }

  if (doCal) {
    if (wb.calibrate(cfg)) {                      // interactive wizard
      if (wb.pushConfig(cfg) && wb.save()) {      // push + persist to EEPROM
        Serial.println(F("saved to EEPROM."));
        delay(50);
        WbConfig rb;
        if (wb.getConfig(rb)) {
          Serial.print(F("readback: gate=")); Serial.print(rb.gate);
          Serial.print(F(" wbound="));         Serial.print(rb.wbound);
          Serial.print(F(" hyst="));           Serial.print(rb.hyst);
          Serial.print(F(" valid="));          Serial.println(wb.configValid());
        }
      } else Serial.println(F("!! push/save failed"));
    } else {
      Serial.println(F("calibration failed; using whatever the PIC holds."));
      wb.getConfig(cfg);
    }
  } else {
    if (wb.getConfig(cfg)) {
      Serial.print(F("using stored config: gate=")); Serial.print(cfg.gate);
      Serial.print(F(" wbound=")); Serial.print(cfg.wbound);
      Serial.print(F(" hyst="));   Serial.println(cfg.hyst);
    }
  }

  // one-shot demo of the remaining getters
  Serial.print(F("tempReady="));  Serial.print(wb.tempReady());
  Serial.print(F(" waterReady=")); Serial.println(wb.waterReady());

  Serial.println(F("t(s)  value  water  sense  ref  tempC"));
}

void loop() {
  long v  = wb.value();     // 0x11
  int  w  = wb.water();     // 0x08 (PIC's decision)
  long sn = wb.sense();     // 0x22
  long rf = wb.ref();       // 0x23
  float tc = wb.tempC();    // 0x07 -> Celsius

  Serial.print(tsec); Serial.print(F("  "));
  if (v < 0 || w < 0) {
    Serial.println(F("MISS"));
    if (++miss >= 5) { wb.recover(); miss = 0; wb.getConfig(cfg); }
  } else {
    miss = 0;
    Serial.print(v);  Serial.print(F("  "));
    Serial.print(w ? F("WATER") : F("dry")); Serial.print(F("  "));
    Serial.print(sn); Serial.print(F("  "));
    Serial.print(rf); Serial.print(F("  "));
    if (!isnan(tc)) Serial.print(tc, 1); else Serial.print(F("--"));
    Serial.println();
  }
  tsec++;
  delay(500);
}
