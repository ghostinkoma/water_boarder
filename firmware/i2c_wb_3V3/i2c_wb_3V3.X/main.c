/* ============================ BUILD: 3.3V (low-range temperature) ============================
 * 5V build -> flash this file; 3.3V build -> flash i2c_wb_3V3.c.  The two files are IDENTICAL
 * except for the title line above and one line: #define TEMP_HIGH_RANGE (1 for 5V, 0 for 3.3V).
 * ** Edit only i2c_wb_5V.c, then regenerate the 3.3V file with:  ./gen_3v3.sh **
 * Host temperature calibration must match this build's range:
 *     5V  high range: setTempCal(4.8, 4.0, 94)   (master TEMP_VDD=4.8/MODE=4.0/CAL=94)
 *     3.3V low  range: setTempCal(3.3, 2.0, N)    (N re-derived: N = idealADC - rawADC at a
 *                      known temp; ideal = (3.3-2*Vt)/3.3*1023, Vt=0.659-(T+40)*0.00132)
 * Water sensing must be (re)calibrated at the operating voltage regardless of build.
 * ==========================================================================================*/
/* PIC12F1840 - WaterBoader COMMAND/REGISTER protocol over hardware I2C (slave 0x20).
 *
 * Builds on the VALIDATED Phase-3 sensor (i2c_touch.c, 45k-sample water test):
 *   - interrupt-driven MSSP I2C (a read never collides with a measurement)
 *   - differential CPS (CPS3/RA4 sense - CPS0/RA0 ref) for EMI/common-mode rejection
 *   - sliding-window trimmed mean (integer only)
 * That measurement pipeline is copied here UNCHANGED.  This file only ADDS a
 * register/command layer on top so the master (and the AquaController) can talk to
 * it cleanly, per the project command spec.
 *
 * ==== I2C REGISTER-POINTER MODEL (idiomatic, like a BME280) ====
 *   WRITE [reg]              -> set the register pointer (persists across transfers)
 *   WRITE [reg][d0..]        -> write data to that register
 *   READ                     -> returns that register bytes (LOW first)
 *   A plain READ with no pointer set returns reg 0x11 (the sensor value).
 * Because the sensor free-runs (BME280 normal mode), Mes and Ready need no handshake:
 * the value is always the latest completed measurement, so Ready* just read 1.
 *
 * ==== REGISTER / COMMAND MAP (spec-aligned; NCO -> CPS on this chip) ====
 *  reg   name (spec)             acc  len  meaning
 *  0x00  ResetDevice             W    0    writing the pointer 0x00 -> soft RESET
 *  0x03  SetWaterBoarderValue    R/W  2    water boundary on the sensor value (ON)
 *  0x04  SetWaterHisWidth        R/W  2    hysteresis width (OFF = boundary + width)
 *  0x05  ReadyTempValue          R    1    1 = ready  (1 when ENABLE_TEMP; else 0)
 *  0x06  ReadyWaterValue         R    1    1 = ready  (always 1)
 *  0x07  GetTempValue            R    2    raw 10-bit die-temp ADC (0..1023, LOW first)
 *  0x08  GetWaterBoarderValue    R    1    1 = water above sensor, 0 = below (hyst)
 *  0x0A  SaveConfig              W    1    write 0xA5 -> save {gate,wbound,hyst,cals} to EEPROM
 *  0x0B  ConfigValid             R    1    1 = valid config was loaded from EEPROM at boot
 *  0x0C  ResetCause              R    1    last reset: b0 POR / b1 BOR / b2 WDT / b3 MCLR / b4 soft-RESET
 *  0x10  ReadyNCO                R    1    1 = ready  (always 1)
 *  0x11  GetNCORaw               R    2    sensor value (windowed diff, biased)  <-default
 *  0x20  (Gate)                  R/W  1    gate length 1..200 (~100us units)
 *  0x21  (OutSel)                R/W  1    0=diff(prod) 1=sense 2=ref (debug output)
 *  0x22  (SenseRaw)              R    2    raw sense count (diagnostic)
 *  0x23  (RefRaw)                R    2    raw ref count (diagnostic)
 *  0x24  CalAir                  R/W  2    calibration ref value: sensor in open air
 *  0x25  CalDry                  R/W  2    calibration ref value: dry glass mounted
 *  0x26  CalWater                R/W  2    calibration ref value: water behind glass
 * (Mes* 0x01/0x02/0x09 are accepted as no-ops: the sensor measures continuously.)
 *
 * ==== EEPROM AUTONOMY (DS 11) ====
 * The master calibrates, pushes gate/wbound/hyst/cals, then writes 0x0A=0xA5 to save
 * them to data EEPROM.  On every boot the PIC reloads them, so it keeps deciding water
 * level correctly with NO master present (redundancy).  EEPROM writes are slow (~5ms/
 * byte) so 0x0A only sets a flag; the actual save runs in main() (never in the ISR).
 * NOTE vs spec: values are 16-bit (2 bytes) here, not the 10F322's 20-bit/3-byte NCO,
 * because the 1840's CPS count and ADC are <= 16-bit.  Temp is left Phase-2 to avoid
 * any risk to the validated CPS (temp uses the FVR; kept off until co-run is proven).
 *
 * PINS: VDD=1, RA5=2(LED), RA4=3(CPS3 SENSE), RA3=4(MCLR/VPP),
 *       RA2=5(SDA), RA1=6(SCL/ICSPCLK), RA0=7(CPS0 REF / ICSPDAT), VSS=8.
 */
#include <xc.h>
#include <stdint.h>

/* Robustness config: WDTE=ON (hang/freeze -> auto reset), BOREN=ON (brown-out reset on
 * voltage sag), PWRTE=ON (power-up settle), STVREN=ON (stack over/underflow -> reset). */
#pragma config FOSC = INTOSC, WDTE = ON, PWRTE = ON, MCLRE = OFF
#pragma config CP = OFF, CPD = OFF, BOREN = ON, CLKOUTEN = OFF
#pragma config IESO = OFF, FCMEN = OFF
#pragma config WRT = OFF, PLLEN = OFF, STVREN = ON, BORV = LO, LVP = OFF

#define _XTAL_FREQ 16000000UL
#define I2C_ADDR   0x20
#define WIN        16
#define CH_SENSE   0x03
#define CH_REF     0x00
#define DIFF_BIAS  20000

/* ===================== PHASE-2 DIE TEMPERATURE (10-bit) =====================
 * Adds die-temp on the ADC's internal temperature-indicator channel (CHS=11101).
 * The ADC is otherwise UNUSED by the CPS path (CPS uses its own oscillator->TMR0),
 * so the temp MUX is connected ONCE at init and never switched again.  One
 * conversion per main() free-run loop (~41ms) => the >=200us recovery requirement
 * (DS 15.3.1) is met naturally.  This was the root cause of the old "value too low"
 * pending bug: continuous back-to-back conversions.  Raw 10-bit is returned over
 * I2C; Celsius (AN1333 Eq.5) is computed on the master.
 *
 *   ENABLE_TEMP     : set 0 to compile out ALL temp/FVR/ADC code and fall back to
 *                     the exact validated CPS-only behavior (clean isolation if the
 *                     shared FVR is ever suspected of disturbing the CPS).
 *   TEMP_HIGH_RANGE : *** the ONLY knob that changes between 5V and 3.3V ***
 *                     1 = high range  (Vtemp = VDD-4Vt) -> needs VDD >= 3.6V; master MODE = 4.0
 *                     0 = low  range  (Vtemp = VDD-2Vt) -> use at 3.3V;       master MODE = 2.0
 *                     Currently running at 5V -> high range.  To migrate to 3.3V,
 *                     flip this to 0 here AND set MODE=2.0 on the master. Nothing
 *                     else in this file changes.
 * ADFVR (ADC FVR buffer, FVRCON<1:0>) is kept OFF: enabling it once skewed the
 * temperature reading (see DISPATCH).  Only TSEN/TSRNG/FVREN are used. */
#define ENABLE_TEMP      1
#define TEMP_HIGH_RANGE  0

#if TEMP_HIGH_RANGE
  #define FVRCON_TEMP    0xB0   /* FVREN=1, TSEN=1, TSRNG=1 (high), ADFVR=00 (off) */
#else
  #define FVRCON_TEMP    0xA0   /* FVREN=1, TSEN=1, TSRNG=0 (low),  ADFVR=00 (off) */
#endif
/* ADCS=111 = dedicated FRC oscillator.  Microchip's own ADC example (DS Example 16-1)
 * uses FRC, and FRC is the recommended clock for the HIGH-IMPEDANCE internal channels
 * (temperature indicator / FVR).  NOTE (2026-07-22): FRC vs FOSC/32 was A/B tested on
 * hardware and made NO difference to the reading -- the clock was NOT the cause of the
 * low reading (that turned out to be this part's intrinsic temp-circuit offset, fixed
 * by one-point calibration on the master).  FRC is kept only because it is the
 * DS-recommended config for internal channels and is otherwise harmless. */
#define ADCON1_TEMP      0xF0   /* ADFM=1 right-justified, ADCS=111 (FRC), VREF=VDD */
#define ADCON0_TEMP      0x75   /* CHS=11101 (temp indicator), ADON=1, GO=0 */

/* register ids */
#define R_RESET    0x00
#define R_WBOUND   0x03
#define R_HYST     0x04
#define R_RDYTEMP  0x05
#define R_RDYWATER 0x06
#define R_GETTEMP  0x07
#define R_GETWATER 0x08
#define R_SAVECFG  0x0A
#define R_CFGVALID 0x0B
#define R_RSTCAUSE 0x0C
#define R_RDYNCO   0x10
#define R_GETNCO   0x11
#define R_GATE     0x20
#define R_OUTSEL   0x21
#define R_SENSE    0x22
#define R_REF      0x23
#define R_CALAIR   0x24
#define R_CALDRY   0x25
#define R_CALWATER 0x26

/* ---- EEPROM layout (data EEPROM, DS 11).  Magic is written LAST on save so a power
 * loss mid-save leaves it absent -> treated as invalid -> compile defaults used. ---- */
#define EE_MAGIC        0xA6   /* bumped from 0xA5: window_estimate overflow fix changed the
                                * value space, so any config saved by older firmware is invalid.
                                * A mismatched magic -> ConfigValid=0 -> master forces re-cal. */
#define EE_A_MAGIC      0
#define EE_A_GATE       1
#define EE_A_WBOUND     2
#define EE_A_HYST       4
#define EE_A_AIR        6
#define EE_A_DRY        8
#define EE_A_WATER     10
#define SAVE_CMD        0xA5   /* data byte that triggers a save when written to 0x0A */

/* ---- shared state (volatile: written by main(), read by the ISR) ---- */
static volatile uint16_t v_value = DIFF_BIAS;          /* windowed sensor value */
static volatile uint16_t v_sense = 0, v_ref = 0;
static volatile uint16_t v_temp  = 0;                  /* trimmed-mean 10-bit die-temp ADC (Phase-2) */
static volatile uint16_t v_wbound = DIFF_BIAS - 250;   /* water ON boundary (SetWaterBoarder) */
static volatile uint16_t v_hyst   = 70;                /* hysteresis width (SetWaterHisWidth) */
static volatile uint8_t  v_water  = 0;                 /* 1 = water present */
static volatile uint8_t  gate     = 20;
static volatile uint8_t  outsel   = 0;                 /* 0=diff 1=sense 2=ref */
static volatile uint8_t  do_reset = 0;
static volatile uint16_t cal_air  = 0, cal_dry = 0, cal_water = 0; /* stored calibration refs */
static volatile uint8_t  config_valid = 0;             /* 1 = loaded from EEPROM at boot */
static volatile uint8_t  do_save  = 0;                 /* ISR sets it; main() does the EEPROM write */
static volatile uint8_t  reset_cause = 0;              /* why the PIC last reset (see 0x0C) */

/* ---- I2C register-pointer bookkeeping (ISR only) ---- */
static volatile uint8_t  reg   = R_GETNCO;             /* current register pointer */
static volatile uint8_t  wpos  = 0;                    /* write byte position within a transfer */
static volatile uint8_t  wtmp  = 0;                    /* first data byte of a 2-byte write */
static volatile uint8_t  rbuf[2];                      /* read response bytes */
static volatile uint8_t  rlen  = 2;
static volatile uint8_t  ridx  = 0;

/* ---- measurement window ---- */
static uint16_t Awin[WIN];
static uint8_t  widx = 0, wfill = 0;

/* ---- data EEPROM byte access (DS 11.2, Examples 11-1 / 11-2) ---- */
static uint8_t ee_read(uint8_t addr)
{
    EEADRL = addr;
    EECON1bits.EEPGD = 0;                               /* data EEPROM, not program */
    EECON1bits.CFGS  = 0;                               /* not config space */
    EECON1bits.RD    = 1;
    return EEDATL;
}

static void ee_write(uint8_t addr, uint8_t data)
{
    EEADRL = addr;
    EEDATL = data;
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS  = 0;
    EECON1bits.WREN  = 1;
    INTCONbits.GIE = 0;                                 /* the unlock must not be interrupted */
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    INTCONbits.GIE = 1;                                 /* re-enable: the ~5ms wait can be interrupted */
    EECON1bits.WREN = 0;
    while (EECON1bits.WR) { }                           /* wait for the cell write to finish */
}

static uint16_t ee_read16(uint8_t a) { return (uint16_t)ee_read(a) | ((uint16_t)ee_read(a + 1) << 8); }
static void     ee_write16(uint8_t a, uint16_t v) { ee_write(a, (uint8_t)v); ee_write(a + 1, (uint8_t)(v >> 8)); }

/* Load saved config at boot; if no valid magic, keep the compile-time defaults. */
static void config_load(void)
{
    if (ee_read(EE_A_MAGIC) == EE_MAGIC) {
        gate      = ee_read(EE_A_GATE);
        v_wbound  = ee_read16(EE_A_WBOUND);
        v_hyst    = ee_read16(EE_A_HYST);
        cal_air   = ee_read16(EE_A_AIR);
        cal_dry   = ee_read16(EE_A_DRY);
        cal_water = ee_read16(EE_A_WATER);
        config_valid = 1;
    }
}

/* Persist the current config.  Magic is written LAST so a partial save reads invalid.
 * config_valid is cleared for the duration so the master can poll ConfigValid (0x0B) and
 * see it return to 1 only when THIS save has actually committed (no early-read race). */
static void config_save(void)
{
    config_valid = 0;                                  /* "commit in progress" */
    CLRWDT();                                          /* the ~60ms EEPROM burst is < 1s WDT, but be safe */
    ee_write(EE_A_GATE, gate);
    ee_write16(EE_A_WBOUND, v_wbound);
    ee_write16(EE_A_HYST,   v_hyst);
    ee_write16(EE_A_AIR,    cal_air);
    ee_write16(EE_A_DRY,    cal_dry);
    ee_write16(EE_A_WATER,  cal_water);
    ee_write(EE_A_MAGIC, EE_MAGIC);
    config_valid = 1;
}

/* Capture why we just reset, BEFORE the first CLRWDT (which would set STATUS.TO=1).
 * PCON bits are active-low (0 = that reset occurred): POR=b1, BOR=b0, RMCLR=b3, RI=b2,
 * STKOVF=b7, STKUNF=b6.  STATUS.TO=0 means a WDT time-out.  We then re-arm POR/BOR. */
static void capture_reset_cause(void)
{
    uint8_t p = PCON, c = 0;
    if (!STATUSbits.nTO)  c |= 0x04;                    /* WDT time-out (hang recovered) */
    if (!(p & 0x02))      c |= 0x01;                    /* POR (power-on) */
    else if (!(p & 0x01)) c |= 0x02;                    /* BOR (brown-out, no POR) */
    if (!(p & 0x08))      c |= 0x08;                    /* RMCLR (MCLR pin) */
    if (!(p & 0x04))      c |= 0x10;                    /* RI (soft RESET instruction) */
    if (p & 0x80)         c |= 0x20;                    /* stack overflow */
    if (p & 0x40)         c |= 0x40;                    /* stack underflow */
    reset_cause = c;
    PCON = 0x0F;                                        /* re-arm POR/BOR/RMCLR/RI, clear stack flags */
}

static void i2c_slave_init(void)
{
    ANSELA = 0x00;
    TRISA  = 0xFF;
    TRISAbits.TRISA5 = 0;  LATAbits.LATA5 = 0;         /* RA5 = LED */

    SSP1STAT = 0x80;
    SSP1ADD  = I2C_ADDR << 1;
    SSP1MSK  = 0xFE;
    SSP1CON1 = 0x26;                                   /* SSPEN + I2C slave 7-bit */
    SSP1CON2 = 0x01;                                   /* SEN=1 clock stretch */
    SSP1CON3 = 0x00;
    PIR1bits.SSP1IF = 0;
    PIE1bits.SSP1IE = 1;
}

static void cps_init(void)
{
    ANSELAbits.ANSA4 = 1;  TRISAbits.TRISA4 = 1;       /* CPS3 sense */
    ANSELAbits.ANSA0 = 1;  TRISAbits.TRISA0 = 1;       /* CPS0 ref   */
    CPSCON1 = CH_SENSE;
    CPSCON0 = 0x88 | 0x01;                             /* CPSON | CPSRNG(10) | T0XCS */
    OPTION_REGbits.TMR0CS = 1;
    OPTION_REGbits.PSA    = 1;
}

static uint16_t cps_measure(uint8_t ch)
{
    uint8_t hi = 0, g;
    CPSCON1 = ch;
    __delay_us(500);
    TMR0 = 0;
    INTCONbits.TMR0IF = 0;
    for (g = gate; g; g--) {
        __delay_us(100);
        if (INTCONbits.TMR0IF) { INTCONbits.TMR0IF = 0; hi++; }
    }
    return (uint16_t)(((uint16_t)hi << 8) | TMR0);
}

#if ENABLE_TEMP
/* Enable FVR + temperature indicator + ADC on the temp channel, once.
 * Leaves the MUX on the temp indicator forever (CPS never touches the ADC). */
static void temp_init(void)
{
    FVRCON = FVRCON_TEMP;
    ADCON1 = ADCON1_TEMP;
    ADCON0 = ADCON0_TEMP;                              /* connects temp MUX + ADON */
    __delay_ms(1);                                     /* FVR + temp-indicator settle (>=200us) */
}

/* One 10-bit conversion.  MUX is fixed on temp, so no re-settle needed; the >=200us
 * inter-conversion recovery is provided by the surrounding main() loop period.
 * PIC returns the RAW 10-bit count; Celsius + one-point calibration are done on the
 * master (AN1333 Eq.5).  Verified 2026-07-22: reading is stable and independent of the
 * ADC clock (FOSC/32 vs FRC) and of the CPS oscillator (on/off) -> the ~88 C absolute
 * offset vs the AN1333 model is this part's intrinsic temp-circuit offset (DS 15: high
 * range "may be less consistent from part to part"), corrected by one-point cal. */
static uint16_t temp_read(void)
{
    ADCON0bits.GO_nDONE = 1;
    while (ADCON0bits.GO_nDONE) { }
    return (uint16_t)(((uint16_t)ADRESH << 8) | ADRESL);  /* right-justified 0..1023 */
}

/* Trimmed mean over the last TWIN temp samples: drop the min AND the max, average
 * the middle 8 (integer only, insertion sort -- same robust estimator the CPS path
 * uses).  Samples enter one-per-main-loop (~41ms apart) so each is naturally >200us
 * from the last (DS 15.3.1) -- no burst, no recovery-time risk.  AN2798 2.1 likewise
 * recommends oversampling/averaging to cut FVR + CPU switching noise. */
#define TWIN 10
static uint16_t Twin[TWIN];
static uint8_t  twidx = 0, twfill = 0;

static uint16_t temp_trimmed(void)
{
    uint16_t s[TWIN]; uint8_t n = twfill, i, j; uint16_t key, sum;
    if (n == 0) return 0;
    for (i = 0; i < n; i++) s[i] = Twin[i];
    for (i = 1; i < n; i++) { key = s[i]; j = i;
        while (j != 0 && s[j-1] > key) { s[j] = s[j-1]; j--; } s[j] = key; }
    if (n < TWIN) return s[n >> 1];                    /* median until the window fills */
    sum = 0; for (i = 1; i < 9; i++) sum += s[i];      /* drop s[0]=min and s[9]=max */
    return (uint16_t)(sum >> 3);                       /* mean of the middle 8 */
}
#endif

static uint16_t window_estimate(void)
{
    uint16_t s[WIN]; uint8_t n = wfill, i, j; uint16_t key;
    uint32_t sum;                                      /* 32-bit: 8 * ~19000 > 65535 */
    if (n == 0) return DIFF_BIAS;
    for (i = 0; i < n; i++) s[i] = Awin[i];
    for (i = 1; i < n; i++) { key = s[i]; j = i;
        while (j != 0 && s[j-1] > key) { s[j] = s[j-1]; j--; } s[j] = key; }
    if (n < WIN) return s[n >> 1];
    sum = 0; for (i = 4; i < 12; i++) sum += s[i];     /* trimmed mean of the middle 8 */
    return (uint16_t)(sum >> 3);
}

/* Fill rbuf/rlen for the current register.  Called at the read address-match. */
static void load_read(void)
{
    ridx = 0;
    switch (reg) {
        case R_GETWATER:  rbuf[0] = v_water;             rlen = 1; break;
#if ENABLE_TEMP
        case R_RDYTEMP:   rbuf[0] = 1;                   rlen = 1; break;
        case R_GETTEMP:   rbuf[0] = (uint8_t)v_temp;  rbuf[1] = (uint8_t)(v_temp >> 8); rlen = 2; break;
#else
        case R_RDYTEMP:   rbuf[0] = 0;                   rlen = 1; break; /* Phase-2 disabled */
        case R_GETTEMP:   rbuf[0] = 0; rbuf[1] = 0;      rlen = 2; break; /* Phase-2 disabled */
#endif
        case R_RDYWATER:  rbuf[0] = 1;                   rlen = 1; break;
        case R_RDYNCO:    rbuf[0] = 1;                   rlen = 1; break;
        case R_CFGVALID:  rbuf[0] = config_valid;        rlen = 1; break;
        case R_RSTCAUSE:  rbuf[0] = reset_cause;         rlen = 1; break;
        case R_GATE:      rbuf[0] = gate;                rlen = 1; break;
        case R_OUTSEL:    rbuf[0] = outsel;              rlen = 1; break;
        case R_CALAIR:    rbuf[0] = (uint8_t)cal_air;   rbuf[1] = (uint8_t)(cal_air   >> 8); rlen = 2; break;
        case R_CALDRY:    rbuf[0] = (uint8_t)cal_dry;   rbuf[1] = (uint8_t)(cal_dry   >> 8); rlen = 2; break;
        case R_CALWATER:  rbuf[0] = (uint8_t)cal_water; rbuf[1] = (uint8_t)(cal_water >> 8); rlen = 2; break;
        case R_WBOUND:    rbuf[0] = (uint8_t)v_wbound; rbuf[1] = (uint8_t)(v_wbound >> 8); rlen = 2; break;
        case R_HYST:      rbuf[0] = (uint8_t)v_hyst;   rbuf[1] = (uint8_t)(v_hyst   >> 8); rlen = 2; break;
        case R_SENSE:     rbuf[0] = (uint8_t)v_sense;  rbuf[1] = (uint8_t)(v_sense  >> 8); rlen = 2; break;
        case R_REF:       rbuf[0] = (uint8_t)v_ref;    rbuf[1] = (uint8_t)(v_ref    >> 8); rlen = 2; break;
        case R_GETNCO:
        default:          rbuf[0] = (uint8_t)v_value;  rbuf[1] = (uint8_t)(v_value  >> 8); rlen = 2; break;
    }
}

/* Apply a completed register write. */
static void apply_write(uint8_t d)
{
    switch (reg) {
        case R_GATE:    if (d) gate = d;                                   break;
        case R_OUTSEL:  if (d <= 2) outsel = d;                            break;
        case R_SAVECFG: if (d == SAVE_CMD) do_save = 1;                    break; /* main() does the write */
        case R_WBOUND:  if (wpos == 2) wtmp = d;
                        else if (wpos == 3) v_wbound  = ((uint16_t)d << 8) | wtmp; break;
        case R_HYST:    if (wpos == 2) wtmp = d;
                        else if (wpos == 3) v_hyst    = ((uint16_t)d << 8) | wtmp; break;
        case R_CALAIR:  if (wpos == 2) wtmp = d;
                        else if (wpos == 3) cal_air   = ((uint16_t)d << 8) | wtmp; break;
        case R_CALDRY:  if (wpos == 2) wtmp = d;
                        else if (wpos == 3) cal_dry   = ((uint16_t)d << 8) | wtmp; break;
        case R_CALWATER:if (wpos == 2) wtmp = d;
                        else if (wpos == 3) cal_water = ((uint16_t)d << 8) | wtmp; break;
        default: break;                                  /* Mes/no-op registers ignore data */
    }
}

static void __interrupt() isr(void)
{
    uint8_t stat, junk, d;
    if (!PIR1bits.SSP1IF) return;
    PIR1bits.SSP1IF = 0;

    if (SSP1CON1bits.SSPOV || SSP1CON1bits.WCOL) {
        junk = SSP1BUF; (void)junk;
        SSP1CON1bits.SSPOV = 0; SSP1CON1bits.WCOL = 0; SSP1CON1bits.CKP = 1;
        return;
    }

    stat = SSP1STAT;
    if ((stat & 0x20) == 0) {                            /* ADDRESS byte */
        junk = SSP1BUF; (void)junk;
        LATAbits.LATA5 ^= 1;
        if (stat & 0x04) {                               /* master READ */
            load_read();
            SSP1BUF = rbuf[ridx++];
        } else {                                         /* master WRITE */
            wpos = 0;
        }
        SSP1CON1bits.CKP = 1;
    } else {                                             /* DATA byte */
        if (stat & 0x04) {                               /* slave transmit (read) */
            if (SSP1CON2bits.ACKSTAT == 0) {             /* master ACKed -> send next byte */
                SSP1BUF = (ridx < rlen) ? rbuf[ridx++] : 0x00;
            }
            /* On master NACK (ACKSTAT==1) the read is done: don't touch SSP1BUF (would WCOL),
             * but ALWAYS release the clock so a stretched SCL can never hang the bus. */
            SSP1CON1bits.CKP = 1;
        } else {                                         /* master wrote a byte */
            d = SSP1BUF;
            if (wpos == 0) {                             /* first byte = register pointer */
                reg = d;
                wpos = 1;
                if (reg == R_RESET) do_reset = 1;        /* ResetDevice */
            } else {
                wpos++;
                apply_write(d);
            }
            SSP1CON1bits.CKP = 1;
        }
    }
}

void main(void)
{
    OSCCON = 0x78;                                       /* 16 MHz HF */
    capture_reset_cause();                               /* must run before the first CLRWDT */
    WDTCON = 0x14;                                        /* WDTPS=01010 -> ~1 s WDT period */
    i2c_slave_init();
    cps_init();
#if ENABLE_TEMP
    temp_init();
#endif
    config_load();                                       /* EEPROM -> gate/wbound/hyst/cals (autonomy) */
    INTCONbits.PEIE = 1;
    INTCONbits.GIE  = 1;

    for (;;) {
        CLRWDT();                                        /* pet the watchdog once per loop (~45ms) */
        uint16_t sense = cps_measure(CH_SENSE);
        uint16_t ref   = cps_measure(CH_REF);
        int      diff  = (int)sense - (int)ref;
        uint16_t a, b;
        uint8_t  w;
#if ENABLE_TEMP
        uint16_t t;
        Twin[twidx] = temp_read();                       /* one conversion per loop (~41ms) */
        if (++twidx >= TWIN) twidx = 0;
        if (twfill < TWIN)   twfill++;
        t = temp_trimmed();                              /* drop min/max, mean of middle 8 */
#endif

        if      (outsel == 1) a = sense;
        else if (outsel == 2) a = ref;
        else                  a = (uint16_t)(DIFF_BIAS + diff);

        Awin[widx] = a;
        if (++widx >= WIN) widx = 0;
        if (wfill < WIN)   wfill++;
        b = window_estimate();

        /* water decision: value drops when water is present (diff more negative).
         * ON when value <= boundary; OFF when value >= boundary + hysteresis. */
        w = v_water;
        if (!w) { if (b <= v_wbound)            w = 1; }
        else    { if (b >= v_wbound + v_hyst)   w = 0; }

        INTCONbits.GIE = 0;                              /* atomic vs the ISR */
        v_value = b;  v_sense = sense;  v_ref = ref;  v_water = w;
#if ENABLE_TEMP
        v_temp  = t;
#endif
        INTCONbits.GIE = 1;

        if (do_save)  { do_save = 0; config_save(); }   /* SaveConfig: ~60ms EEPROM write, main-context */
        if (do_reset) { __delay_ms(2); asm("reset"); }  /* ResetDevice command */
    }
}
