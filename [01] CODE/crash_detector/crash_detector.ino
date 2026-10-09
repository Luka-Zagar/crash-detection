/*
  Crash detector - DAVI M300 laser head on ABB robot
  ---------------------------------------------------
  2x 20 kg load cell -> ADS1256 (PGA 64, buffer on) -> Arduino Nano -> D7 -> DST opto -> ABB DI

  Wiring (Nano):
    D13 SCLK | D11 DIN | D12 DOUT | D10 CS | D2 DRDY | D8 PDWN | D7 -> opto IN1
    Load cell X: AIN0 (+) / AIN1 (-)     Load cell Y: AIN2 (+) / AIN3 (-)

  Output logic (FAIL-SAFE):
    D7 HIGH = OK, robot may run
    D7 LOW  = CRASH / FAULT / booting  -> ABB input = 0 -> stop robot + laser off
  A crash is LATCHED until the Nano is reset (reset button, power cycle,
  or opening the serial port, which also resets a Nano).

  Serial 115200 baud:
    Serial Plotter shows X, Y and the running peaks (Xmax, Xmin, Ymax, Ymin).
    Commands (type in Serial Monitor / plotter input field):
      p  print peaks since reset + suggested thresholds (peak x MARGIN)
      r  reset peaks
      t  re-tare (robot must be standing still; not possible after a crash)

  Learning procedure:
    1. Robot in home pose, power/reset the Nano (it tares at startup).
    2. Run all robot programs a few times.
    3. Send 'p', copy the suggested THR_ lines below, upload again.
*/

#include <SPI.h>
#include <avr/wdt.h>

// ============================ USER SETTINGS ============================

// Crash thresholds per direction, same units as the plot (~N at the load cell).
// All values are POSITIVE magnitudes: X- triggers when X < -THR_X_NEG.
const float THR_X_POS = 30.0;
const float THR_X_NEG = 30.0;
const float THR_Y_POS = 30.0;
const float THR_Y_NEG = 30.0;

const float MARGIN        = 1.25;  // suggested threshold = learned peak * MARGIN
const float MIN_THRESHOLD = 5.0;   // never suggest less than this (noise floor)

// Scale in ADC counts per unit. 2736 counts ~ 1 N for a 1.0 mV/V 20 kg cell,
// 5 V excitation, PGA 64. To calibrate: hang a known mass in the axis direction,
// read the value with scale = 1.0 (raw counts), then scale = counts / (kg * 9.81).
// A NEGATIVE scale flips the direction of that axis.
const float COUNTS_PER_UNIT_X = 2736.0;
const float COUNTS_PER_UNIT_Y = 2736.0;

const uint8_t  CONFIRM_SAMPLES = 3;    // consecutive samples over threshold before trigger (~5 ms)
const uint16_t TARE_SAMPLES    = 400;  // samples per axis averaged for tare (~0.7 s)
const uint8_t  PLOT_EVERY      = 10;   // plot every Nth X/Y pair (~60 lines/s)
const uint16_t CRASH_WINDOW_MS = 300;  // how long after trigger the crash peak is captured

const uint8_t OUT_OK_LEVEL = HIGH;     // fail-safe: HIGH = OK. Swap only if wiring requires it.

// ============================ PINS ============================
const uint8_t PIN_CS   = 10;
const uint8_t PIN_DRDY = 2;
const uint8_t PIN_PDWN = 8;
const uint8_t PIN_OUT  = 7;

const uint8_t OUT_STOP_LEVEL = (OUT_OK_LEVEL == HIGH) ? LOW : HIGH;

// ============================ ADS1256 ============================
#define CMD_WAKEUP  0x00
#define CMD_RDATA   0x01
#define CMD_RREG    0x10
#define CMD_WREG    0x50
#define CMD_SDATAC  0x0F
#define CMD_SELFCAL 0xF0
#define CMD_SYNC    0xFC
#define CMD_RESET   0xFE

#define REG_STATUS  0x00
#define REG_MUX     0x01
#define REG_ADCON   0x02
#define REG_DRATE   0x03

const uint8_t MUX_X        = 0x01;  // AIN0 - AIN1
const uint8_t MUX_Y        = 0x23;  // AIN2 - AIN3
const uint8_t STATUS_VAL   = 0x02;  // MSB first, auto-cal off, input buffer ON
const uint8_t ADCON_VAL    = 0x06;  // CLKOUT off, sensor detect off, PGA = 64
const uint8_t DRATE_VAL    = 0xB0;  // 2000 SPS (~600 samples/s per axis when switching)

const uint32_t DRDY_TIMEOUT_US = 20000UL;
const long     SAT_LIMIT       = 0x7FFF00L;  // near full scale -> overload or broken wire

SPISettings adsSpi(1000000, MSBFIRST, SPI_MODE1);

// ============================ STATE ============================
const float scaleF[2] = { COUNTS_PER_UNIT_X, COUNTS_PER_UNIT_Y };
const float thr[4]    = { THR_X_POS, THR_X_NEG, THR_Y_POS, THR_Y_NEG };
const char* DIR_NAME[4] = { "X+", "X-", "Y+", "Y-" };

uint8_t convCh = 0;          // axis whose conversion is currently running
long    offset[2] = { 0, 0 };
float   force[2]  = { 0, 0 };
float   peakPos[2], peakNeg[2];
uint8_t overCnt[4] = { 0, 0, 0, 0 };

bool     crashed      = false;
bool     faulted      = false;
bool     reportDone   = false;
uint8_t  crashDir     = 0;
float    crashTrigger = 0;
float    crashPeak[2] = { 0, 0 };
uint32_t crashTime    = 0;
uint8_t  plotDiv      = 0;

// ============================ ADS1256 LOW LEVEL ============================
bool waitDRDY(uint32_t timeout_us) {
  uint32_t t0 = micros();
  while (digitalRead(PIN_DRDY) == HIGH) {
    if (micros() - t0 > timeout_us) return false;
  }
  return true;
}

void adsCommand(uint8_t cmd) {
  SPI.beginTransaction(adsSpi);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(cmd);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(5);
}

void adsWriteReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(adsSpi);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_WREG | reg);
  SPI.transfer(0x00);            // write 1 register
  SPI.transfer(val);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(10);
}

uint8_t adsReadReg(uint8_t reg) {
  SPI.beginTransaction(adsSpi);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_RREG | reg);
  SPI.transfer(0x00);            // read 1 register
  delayMicroseconds(7);          // t6
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(5);
  return v;
}

// Call when DRDY is low: switches MUX to nextMux, starts its conversion,
// and returns the result of the conversion that just finished (previous MUX).
long adsSwitchAndRead(uint8_t nextMux) {
  SPI.beginTransaction(adsSpi);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_WREG | REG_MUX);
  SPI.transfer(0x00);
  SPI.transfer(nextMux);
  delayMicroseconds(2);
  SPI.transfer(CMD_SYNC);
  delayMicroseconds(5);          // t11
  SPI.transfer(CMD_WAKEUP);
  delayMicroseconds(2);
  SPI.transfer(CMD_RDATA);
  delayMicroseconds(7);          // t6
  uint8_t b2 = SPI.transfer(0x00);
  uint8_t b1 = SPI.transfer(0x00);
  uint8_t b0 = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  long v = ((long)b2 << 16) | ((long)b1 << 8) | (long)b0;
  if (v & 0x800000L) v |= 0xFF000000L;  // sign-extend 24 -> 32 bit
  return v;
}

bool adsInit() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_DRDY, INPUT);
  pinMode(PIN_PDWN, OUTPUT);
  digitalWrite(PIN_PDWN, HIGH);
  SPI.begin();
  delay(100);

  adsCommand(CMD_RESET);
  delay(5);
  if (!waitDRDY(500000UL)) return false;
  adsCommand(CMD_SDATAC);

  adsWriteReg(REG_STATUS, STATUS_VAL);
  adsWriteReg(REG_ADCON, ADCON_VAL);
  adsWriteReg(REG_DRATE, DRATE_VAL);
  adsWriteReg(REG_MUX, MUX_X);

  // Verify SPI communication by reading registers back
  if (adsReadReg(REG_DRATE) != DRATE_VAL) return false;
  if (adsReadReg(REG_MUX) != MUX_X) return false;

  adsCommand(CMD_SELFCAL);
  delay(5);
  if (!waitDRDY(1000000UL)) return false;

  adsCommand(CMD_SYNC);
  delayMicroseconds(5);
  adsCommand(CMD_WAKEUP);
  convCh = 0;
  return true;
}

// Gets the next sample. ch = axis it belongs to (0 = X, 1 = Y).
bool getSample(uint8_t &ch, long &raw) {
  if (!waitDRDY(DRDY_TIMEOUT_US)) return false;
  uint8_t next = convCh ^ 1;
  raw = adsSwitchAndRead(next == 0 ? MUX_X : MUX_Y);
  ch = convCh;
  convCh = next;
  return true;
}

// ============================ LOGIC ============================
void latchFault(const __FlashStringHelper* msg) {
  digitalWrite(PIN_OUT, OUT_STOP_LEVEL);
  crashed = true;
  if (!faulted) {
    faulted = true;
    Serial.println();
    Serial.print(F("!!! FAULT: "));
    Serial.println(msg);
    Serial.println(F("    Output latched to STOP. Reset Nano to retry."));
  }
}

void resetPeaks() {
  for (uint8_t i = 0; i < 2; i++) { peakPos[i] = 0; peakNeg[i] = 0; }
}

bool tare() {
  int64_t sum[2] = { 0, 0 };
  uint16_t n[2] = { 0, 0 };
  uint8_t ch;
  long raw;

  for (uint8_t i = 0; i < 10; i++) {         // discard first samples
    if (!getSample(ch, raw)) return false;
  }
  while (n[0] < TARE_SAMPLES || n[1] < TARE_SAMPLES) {
    if (!getSample(ch, raw)) return false;
    if (n[ch] < TARE_SAMPLES) { sum[ch] += raw; n[ch]++; }
    wdt_reset();
  }
  offset[0] = (long)(sum[0] / TARE_SAMPLES);
  offset[1] = (long)(sum[1] / TARE_SAMPLES);
  return true;
}

void triggerCrash(uint8_t dir, float f) {
  digitalWrite(PIN_OUT, OUT_STOP_LEVEL);    // act first, report later
  crashed      = true;
  crashDir     = dir;
  crashTrigger = f;
  crashTime    = millis();
  crashPeak[0] = force[0];
  crashPeak[1] = force[1];
}

void checkThreshold(uint8_t ch, float f) {
  uint8_t iPos = ch * 2;
  uint8_t iNeg = ch * 2 + 1;

  if (f > thr[iPos])  { if (overCnt[iPos] < 255) overCnt[iPos]++; } else overCnt[iPos] = 0;
  if (f < -thr[iNeg]) { if (overCnt[iNeg] < 255) overCnt[iNeg]++; } else overCnt[iNeg] = 0;

  if (!crashed) {
    if (overCnt[iPos] >= CONFIRM_SAMPLES)      triggerCrash(iPos, f);
    else if (overCnt[iNeg] >= CONFIRM_SAMPLES) triggerCrash(iNeg, f);
  }
}

void printCrashReport() {
  Serial.println();
  Serial.print(F("!!! CRASH  dir="));
  Serial.print(DIR_NAME[crashDir]);
  Serial.print(F("  trigger="));
  Serial.print(crashTrigger, 1);
  Serial.print(F("  peakX="));
  Serial.print(crashPeak[0], 1);
  Serial.print(F("  peakY="));
  Serial.print(crashPeak[1], 1);
  Serial.println(F("  -> output latched, reset Nano to re-arm"));
}

void printSuggestion(const __FlashStringHelper* name, float peakMag) {
  float s = peakMag * MARGIN;
  if (s < MIN_THRESHOLD) s = MIN_THRESHOLD;
  Serial.print(F("const float "));
  Serial.print(name);
  Serial.print(F(" = "));
  Serial.print(s, 1);
  Serial.println(F(";"));
}

void printPeaks() {
  Serial.println();
  Serial.println(F("--- Peaks since reset ---"));
  Serial.print(F("X+ ")); Serial.print(peakPos[0], 1);
  Serial.print(F("   X- ")); Serial.print(peakNeg[0], 1);
  Serial.print(F("   Y+ ")); Serial.print(peakPos[1], 1);
  Serial.print(F("   Y- ")); Serial.println(peakNeg[1], 1);
  Serial.println(F("--- Suggested thresholds (copy into code) ---"));
  printSuggestion(F("THR_X_POS"),  peakPos[0]);
  printSuggestion(F("THR_X_NEG"), -peakNeg[0]);
  printSuggestion(F("THR_Y_POS"),  peakPos[1]);
  printSuggestion(F("THR_Y_NEG"), -peakNeg[1]);
}

void plot() {
  Serial.print(F("X:"));     Serial.print(force[0], 1);
  Serial.print(F(",Y:"));    Serial.print(force[1], 1);
  Serial.print(F(",Xmax:")); Serial.print(peakPos[0], 1);
  Serial.print(F(",Xmin:")); Serial.print(peakNeg[0], 1);
  Serial.print(F(",Ymax:")); Serial.print(peakPos[1], 1);
  Serial.print(F(",Ymin:")); Serial.println(peakNeg[1], 1);
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'p') {
      printPeaks();
    } else if (c == 'r') {
      resetPeaks();
      Serial.println(F("Peaks reset."));
    } else if (c == 't') {
      if (crashed) {
        Serial.println(F("Crashed/fault - reset Nano first."));
      } else {
        digitalWrite(PIN_OUT, OUT_STOP_LEVEL);   // no detection during tare -> hold stop
        Serial.println(F("Taring..."));
        if (tare()) {
          resetPeaks();
          digitalWrite(PIN_OUT, OUT_OK_LEVEL);
          Serial.println(F("Tare done."));
        } else {
          latchFault(F("ADS1256 timeout during tare"));
        }
      }
    }
  }
}

// ============================ SETUP / LOOP ============================
void setup() {
  MCUSR = 0;
  wdt_disable();

  pinMode(PIN_OUT, OUTPUT);
  digitalWrite(PIN_OUT, OUT_STOP_LEVEL);       // hold STOP until ready

  Serial.begin(115200);
  Serial.println(F("Crash detector starting..."));

  if (!adsInit()) {
    latchFault(F("ADS1256 not responding (check SPI wiring / power)"));
  } else if (!tare()) {
    latchFault(F("ADS1256 timeout during tare"));
  } else {
    resetPeaks();
    digitalWrite(PIN_OUT, OUT_OK_LEVEL);
    Serial.println(F("Tare done. ARMED."));
  }

  wdt_enable(WDTO_250MS);
}

void loop() {
  wdt_reset();
  handleSerial();

  uint8_t ch;
  long raw;
  if (!getSample(ch, raw)) {
    latchFault(F("ADS1256 DRDY timeout"));
    return;
  }
  if (raw >= SAT_LIMIT || raw <= -SAT_LIMIT) {
    latchFault(F("ADC saturated (overload or broken sensor wire)"));
  }

  float f = (float)(raw - offset[ch]) / scaleF[ch];
  force[ch] = f;

  if (f > peakPos[ch]) peakPos[ch] = f;
  if (f < peakNeg[ch]) peakNeg[ch] = f;

  checkThreshold(ch, f);

  // Capture the peak during the crash window, then report once
  if (crashed && !faulted && !reportDone) {
    if (fabs(f) > fabs(crashPeak[ch])) crashPeak[ch] = f;
    if (millis() - crashTime > CRASH_WINDOW_MS) {
      printCrashReport();
      reportDone = true;
    }
  }

  if (ch == 1 && ++plotDiv >= PLOT_EVERY) {
    plotDiv = 0;
    plot();
  }
}
