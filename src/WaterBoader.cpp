/* WaterBoader.cpp - implementation.  See WaterBoader.h. */
#include "WaterBoader.h"
#include <Wire.h>
#include <ctype.h>
#include <math.h>

/* calibration wizard tunables */
#define WB_CAL_RECORDS  50
#define WB_CAL_REC_MS   150
#define WB_CAL_GATE     200
#define WB_HYST_K       0.7f     /* hysteresis = HYST_K * sigma_dry (0.5..0.8) */
#define WB_HYST_FLOOR   2

WaterBoader::WaterBoader(uint8_t addr)
: _addr(addr), _tvdd(4.8f), _tmode(4.0f), _tcal(94) {}

void WaterBoader::begin(bool avrPullups) {
#if defined(ARDUINO_ARCH_AVR)
  if (avrPullups) { pinMode(A4, INPUT_PULLUP); pinMode(A5, INPUT_PULLUP); }
#else
  (void)avrPullups;
#endif
}

/* ---------------- low-level I2C (retried) ---------------- */
int WaterBoader::read8(uint8_t reg) {
  for (uint8_t t = 0; t < WB_RETRY; t++) {
    Wire.beginTransmission(_addr); Wire.write(reg);
    if (Wire.endTransmission() == 0 && Wire.requestFrom((int)_addr, 1) >= 1)
      return Wire.read();
  }
  return -1;
}
long WaterBoader::read16(uint8_t reg) {
  for (uint8_t t = 0; t < WB_RETRY; t++) {
    Wire.beginTransmission(_addr); Wire.write(reg);
    if (Wire.endTransmission() == 0 && Wire.requestFrom((int)_addr, 2) >= 2) {
      uint16_t lo = Wire.read(), hi = Wire.read();
      return (long)(lo | (hi << 8));
    }
  }
  return -1;
}
bool WaterBoader::write8(uint8_t reg, uint8_t v) {
  for (uint8_t t = 0; t < WB_RETRY; t++) {
    Wire.beginTransmission(_addr); Wire.write(reg); Wire.write(v);
    if (Wire.endTransmission() == 0) return true;
  }
  return false;
}
bool WaterBoader::write16(uint8_t reg, uint16_t v) {
  for (uint8_t t = 0; t < WB_RETRY; t++) {
    Wire.beginTransmission(_addr); Wire.write(reg);
    Wire.write((uint8_t)(v & 0xFF)); Wire.write((uint8_t)(v >> 8));
    if (Wire.endTransmission() == 0) return true;
  }
  return false;
}

/* ---------------- measurements ---------------- */
long WaterBoader::value()      { return read16(WB_R_GETNCO); }
int  WaterBoader::water()      { return read8 (WB_R_GETWATER); }
long WaterBoader::sense()      { return read16(WB_R_SENSE); }
long WaterBoader::ref()        { return read16(WB_R_REF); }
long WaterBoader::tempRaw()    { return read16(WB_R_GETTEMP); }
int  WaterBoader::tempReady()  { return read8 (WB_R_RDYTEMP); }
int  WaterBoader::waterReady() { return read8 (WB_R_RDYWATER); }

float WaterBoader::tempC() {
  long adc = tempRaw();
  if (adc < 0) return NAN;
  float a = (float)(adc + _tcal);
  return (0.659f - (_tvdd / _tmode) * (1.0f - a / 1023.0f)) / 0.00132f - 40.0f;
}
void WaterBoader::setTempCal(float vdd, float mode, int offset) {
  _tvdd = vdd; _tmode = mode; _tcal = offset;
}

/* ---------------- configuration ---------------- */
bool WaterBoader::setBoundary(uint16_t v)   { return write16(WB_R_WBOUND, v); }
bool WaterBoader::setHysteresis(uint16_t v) { return write16(WB_R_HYST, v); }
bool WaterBoader::setGate(uint8_t g)        { return write8 (WB_R_GATE, g); }
bool WaterBoader::setOutSel(uint8_t s)      { return write8 (WB_R_OUTSEL, s); }
bool WaterBoader::setCal(uint16_t air, uint16_t dry, uint16_t water) {
  return write16(WB_R_CALAIR, air) && write16(WB_R_CALDRY, dry) && write16(WB_R_CALWATER, water);
}
bool WaterBoader::getConfig(WbConfig& c) {
  int  g  = read8(WB_R_GATE);
  long wb = read16(WB_R_WBOUND), hy = read16(WB_R_HYST);
  long a  = read16(WB_R_CALAIR), d = read16(WB_R_CALDRY), w = read16(WB_R_CALWATER);
  if (g < 0 || wb < 0 || hy < 0 || a < 0 || d < 0 || w < 0) return false;
  c.gate = (uint8_t)g; c.wbound = (uint16_t)wb; c.hyst = (uint16_t)hy;
  c.calAir = (uint16_t)a; c.calDry = (uint16_t)d; c.calWater = (uint16_t)w;
  return true;
}
bool WaterBoader::pushConfig(const WbConfig& c) {
  bool ok = true;
  ok &= setGate(c.gate);
  ok &= setBoundary(c.wbound);
  ok &= setHysteresis(c.hyst);
  ok &= setCal(c.calAir, c.calDry, c.calWater);
  return ok;
}

/* ---------------- EEPROM / autonomy ---------------- */
bool WaterBoader::save() {
  if (!write8(WB_R_SAVECFG, WB_SAVE_CMD)) return false;
  // The PIC writes EEPROM in its main loop (~100ms) and holds ConfigValid=0 until this
  // save commits.  Poll until it returns 1 (or time out) so save() confirms persistence.
  for (uint8_t i = 0; i < 40; i++) {          // up to ~400ms
    delay(10);
    if (configValid() == 1) return true;
  }
  return false;
}
int  WaterBoader::configValid() { return read8(WB_R_CFGVALID); }

/* ---------------- health / recovery ---------------- */
int  WaterBoader::resetCause()  { return read8(WB_R_RSTCAUSE); }
bool WaterBoader::reset()       { return write8(WB_R_RESET, 0); }

void WaterBoader::printResetCause(uint8_t rc, Stream& io) {
  io.print(F("reset cause:"));
  if (rc & WB_RST_POR)  io.print(F(" POR"));
  if (rc & WB_RST_BOR)  io.print(F(" BOR"));
  if (rc & WB_RST_WDT)  io.print(F(" WDT"));
  if (rc & WB_RST_MCLR) io.print(F(" MCLR"));
  if (rc & WB_RST_SOFT) io.print(F(" soft-RESET"));
  if (rc == 0)          io.print(F(" (none)"));
  io.println();
}

bool WaterBoader::recover(Stream& io) {
  io.println(F("[WaterBoader] recover: requesting reset"));
  reset();                       /* works if the PIC main loop is alive */
  delay(1300);                   /* also covers a ~1s WDT auto-reset of a hung PIC */
  int rc = resetCause();
  if (rc < 0) { io.println(F("[WaterBoader] still unresponsive!")); return false; }
  io.print(F("[WaterBoader] back. "));
  printResetCause((uint8_t)rc, io);
  io.print(F("[WaterBoader] ConfigValid=")); io.println(configValid());
  return true;
}

/* ---------------- calibration wizard ---------------- */
char WaterBoader::_waitChar(Stream& io, const char* valid, unsigned long toMs) {
  while (io.available()) io.read();
  unsigned long t0 = millis();
  for (;;) {
    if (io.available()) {
      char c = (char)tolower(io.read());
      for (const char* p = valid; *p; p++) if ((char)tolower(*p) == c) return c;
    }
    if (toMs && (millis() - t0) >= toMs) return 0;
  }
}

uint16_t WaterBoader::_capture(Stream& io, const __FlashStringHelper* name, uint16_t& sigmaOut) {
  uint16_t buf[WB_CAL_RECORDS];
  io.print(F("  capturing ")); io.print(name); io.print(F(" ..."));
  uint8_t n = 0;
  for (uint8_t i = 0; i < WB_CAL_RECORDS; i++) {
    long v = value();
    if (v >= 0) buf[n++] = (uint16_t)v;
    delay(WB_CAL_REC_MS);
  }
  if (n == 0) { sigmaOut = 0; io.println(F(" NO DATA")); return 0; }
  for (uint8_t i = 1; i < n; i++) {                 /* insertion sort */
    uint16_t k = buf[i]; int j = (int)i - 1;
    while (j >= 0 && buf[j] > k) { buf[j + 1] = buf[j]; j--; }
    buf[j + 1] = k;
  }
  uint16_t med = buf[n / 2];
  double mean = 0; for (uint8_t i = 0; i < n; i++) mean += buf[i]; mean /= n;
  double var  = 0; for (uint8_t i = 0; i < n; i++) { double d = buf[i] - mean; var += d * d; }
  var /= n;
  sigmaOut = (uint16_t)(sqrt(var) + 0.5);
  io.print(F(" median=")); io.print(med);
  io.print(F(" sigma="));  io.print(sigmaOut);
  io.print(F(" (n="));     io.print(n); io.println(F(")"));
  return med;
}

bool WaterBoader::calibrate(WbConfig& out, Stream& io) {
  io.println();
  io.println(F("# WaterBoader calibration (a few minutes)"));
  setGate(WB_CAL_GATE);

  uint16_t air = 0, dry = 0, water = 0, sAir = 0, sDry = 0, sWater = 0;
  uint8_t step = 1;
  while (step <= 3) {
    if (step == 1) {
      io.println(F("(1) keep everything AWAY from the sensor, then press y"));
      _waitChar(io, "y", 0);
      air = _capture(io, F("air"), sAir);
      step = 2;
    } else if (step == 2) {
      io.println(F("(2) mount the sensor on DRY glass, press y  (n = redo step 1)"));
      if (_waitChar(io, "yn", 0) == 'n') { step = 1; continue; }
      dry = _capture(io, F("dry-glass"), sDry);
      step = 3;
    } else {
      io.println(F("(3) put WATER behind the glass, press y  (n = back to step 2)"));
      if (_waitChar(io, "yn", 0) == 'n') { step = 2; continue; }
      water = _capture(io, F("water"), sWater);
      step = 4;
    }
  }

  if (dry <= water) {
    io.println(F("!! dry <= water : states do not separate. aborted."));
    return false;
  }
  uint16_t gap = dry - water;
  uint16_t hyst = (uint16_t)(WB_HYST_K * (float)sDry + 0.5f);
  if (hyst < WB_HYST_FLOOR) hyst = WB_HYST_FLOOR;

  out.gate     = WB_CAL_GATE;
  out.wbound   = (uint16_t)(((uint32_t)dry + water) / 2);
  out.hyst     = hyst;
  out.calAir   = air;
  out.calDry   = dry;
  out.calWater = water;

  io.print(F("gap=")); io.print(gap);
  io.print(F(" sigma_dry=")); io.print(sDry);
  if (sDry && gap < (uint16_t)(4UL * sDry)) io.println(F("  [warn] separation < 4 sigma"));
  else io.println();
  io.print(F("-> wbound=")); io.print(out.wbound);
  io.print(F(" hyst="));     io.print(out.hyst);
  io.print(F("  (air="));    io.print(air);
  io.print(F(" dry="));      io.print(dry);
  io.print(F(" water="));    io.print(water); io.println(F(")"));
  return true;
}
