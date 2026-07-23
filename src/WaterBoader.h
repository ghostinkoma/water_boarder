/*
 * WaterBoader.h - Arduino/ESP32 driver for the PIC12F1840 "WaterBoader" non-contact
 * capacitive water-level sensor (I2C slave, default address 0x20).
 *
 * Pairs with the PIC firmware  i2c_wb_5V.c (5V) / i2c_wb_3V3.c (3.3V).  Drop this folder into your project's library
 * path (e.g. AquaController) and:
 *
 *     #include <WaterBoader.h>
 *     WaterBoader wb;              // default 0x20
 *     void setup() { Wire.begin(); wb.begin(); }
 *     void loop()  { if (wb.water() == 1) alarm(); }
 *
 * The PIC measures a DIFFERENTIAL capacitive value (sense electrode - reference
 * electrode) so common-mode drift (temperature, humidity, VDD) cancels, decides
 * water present/absent against a stored boundary+hysteresis, and can persist its
 * calibration in EEPROM so it keeps working with no master (autonomy).
 *
 * MIT-style: use freely.  (c) 2026 WaterBoader project.
 */
#ifndef WATERBOADER_H
#define WATERBOADER_H
#include <Arduino.h>

/* ---- I2C register map (matches i2c_wb_5V.c / i2c_wb_3V3.c) ---- */
enum WbReg : uint8_t {
  WB_R_RESET     = 0x00,  /* W1 : write anything -> soft reset (asm reset) */
  WB_R_WBOUND    = 0x03,  /* RW2: water ON boundary (sensor-value units)   */
  WB_R_HYST      = 0x04,  /* RW2: hysteresis width (OFF = boundary + hyst) */
  WB_R_RDYTEMP   = 0x05,  /* R1 : 1 = temperature ready                    */
  WB_R_GETTEMP   = 0x07,  /* R2 : raw 10-bit die-temp ADC                  */
  WB_R_GETWATER  = 0x08,  /* R1 : 1 = water present, 0 = dry               */
  WB_R_SAVECFG   = 0x0A,  /* W1 : write 0xA5 -> save config to EEPROM      */
  WB_R_CFGVALID  = 0x0B,  /* R1 : 1 = valid EEPROM config loaded at boot   */
  WB_R_RSTCAUSE  = 0x0C,  /* R1 : reset cause bitfield (see WbReset*)       */
  WB_R_RDYWATER  = 0x06,  /* R1 : 1 = water value ready                    */
  WB_R_RDYNCO    = 0x10,  /* R1 : 1 = sensor value ready                   */
  WB_R_GETNCO    = 0x11,  /* R2 : windowed differential value (default read)*/
  WB_R_GATE      = 0x20,  /* RW1: gate length ~100us units (1..200)        */
  WB_R_OUTSEL    = 0x21,  /* RW1: 0=diff(prod) 1=sense 2=ref (debug)       */
  WB_R_SENSE     = 0x22,  /* R2 : raw sense count                          */
  WB_R_REF       = 0x23,  /* R2 : raw reference count                      */
  WB_R_CALAIR    = 0x24,  /* RW2: calibration value - open air             */
  WB_R_CALDRY    = 0x25,  /* RW2: calibration value - dry glass            */
  WB_R_CALWATER  = 0x26   /* RW2: calibration value - water behind glass   */
};

/* reset-cause bits (reg 0x0C) */
#define WB_RST_POR   0x01
#define WB_RST_BOR   0x02
#define WB_RST_WDT   0x04    /* firmware hang recovered by the watchdog */
#define WB_RST_MCLR  0x08
#define WB_RST_SOFT  0x10    /* soft RESET instruction (our reset()) */

#define WB_SAVE_CMD  0xA5    /* value that triggers an EEPROM save */
#define WB_RETRY     3       /* I2C attempts before an op is reported failed */

/* full device configuration (thresholds + stored calibration references) */
struct WbConfig {
  uint8_t  gate;
  uint16_t wbound;
  uint16_t hyst;
  uint16_t calAir;
  uint16_t calDry;
  uint16_t calWater;
};

class WaterBoader {
public:
  explicit WaterBoader(uint8_t addr = 0x20);

  /* call Wire.begin() yourself first (shared bus); begin() only stores state and,
   * on classic AVR, can enable A4/A5 pull-ups.  Safe to call on ESP32 (no-op pull-ups). */
  void begin(bool avrPullups = false);

  /* ---------- live measurements (all return -1 / NAN on I2C failure) ---------- */
  long  value();        /* 0x11 windowed differential value */
  int   water();        /* 0x08 : 1 water, 0 dry, -1 error  */
  long  sense();        /* 0x22 */
  long  ref();          /* 0x23 */
  long  tempRaw();      /* 0x07 raw 10-bit */
  float tempC();        /* Celsius (AN1333 Eq.5 + one-point cal); NAN on error */
  int   tempReady();    /* 0x05 */
  int   waterReady();   /* 0x06 */

  /* ---------- configuration (PIC decides water on these) ---------- */
  bool  setBoundary(uint16_t v);     /* 0x03 */
  bool  setHysteresis(uint16_t v);   /* 0x04 */
  bool  setGate(uint8_t g);          /* 0x20 */
  bool  setOutSel(uint8_t s);        /* 0x21 : 0=diff 1=sense 2=ref */
  bool  setCal(uint16_t air, uint16_t dry, uint16_t water); /* 0x24-26 */
  bool  getConfig(WbConfig& c);
  bool  pushConfig(const WbConfig& c);

  /* ---------- EEPROM persistence / autonomy ---------- */
  bool  save();          /* 0x0A = 0xA5 -> PIC writes EEPROM */
  int   configValid();   /* 0x0B : 1 if config was loaded from EEPROM at boot */

  /* ---------- health / recovery ---------- */
  int   resetCause();                                /* 0x0C (-1 err) */
  void  printResetCause(uint8_t rc, Stream& io = Serial);
  bool  reset();                                     /* 0x00 soft reset */
  bool  recover(Stream& io = Serial);                /* reset + read why + reconfirm */

  /* ---------- interactive 3-state calibration wizard ---------- */
  /* Prompts on `io`, captures air/dry/water (50 samples each), derives
   * wbound=midpoint(dry,water), hyst=0.7*sigma_dry, fills `out`.  Does NOT push/save;
   * caller does pushConfig()+save() so the flow is explicit.  Returns false if the
   * states do not separate (dry <= water). */
  bool  calibrate(WbConfig& out, Stream& io = Serial);

  /* ---------- temperature calibration (per-part / 3.3V) ----------
   * Defaults: VDD=4.8, mode=4 (high range), offset=94.  For 3.3V use vdd=3.3, mode=2
   * (PIC firmware must be built with TEMP_HIGH_RANGE=0) and re-derive offset. */
  void  setTempCal(float vdd, float mode, int offset);

  /* ---------- low-level access (advanced) ---------- */
  int   read8(uint8_t reg);
  long  read16(uint8_t reg);
  bool  write8(uint8_t reg, uint8_t v);
  bool  write16(uint8_t reg, uint16_t v);

private:
  uint8_t _addr;
  float   _tvdd;
  float   _tmode;
  int     _tcal;
  char     _waitChar(Stream& io, const char* valid, unsigned long toMs);
  uint16_t _capture(Stream& io, const __FlashStringHelper* name, uint16_t& sigmaOut);
};

#endif
