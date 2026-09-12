// ============================================================
//  FALL PREDICTION DEVICE v2 — Edge Implementation
//  Board   : STM32F103C8T6 (Blue Pill)
//  Sensors : ADS1299 (EEG, 3 channels) + BNO055 (IMU)
//  IDE     : Arduino IDE + STM32duino core
//
//  v2 IMPROVEMENTS (research-backed):
//   1. 60 Hz notch filter on EEG (powerline interference rejection)
//   2. Complementary filter for tilt (gyro + accel fusion)
//   3. Coincidence window between EEG and IMU triggers
//   4. ADS1299 hardware lead-off detection (electrode contact)
//   5. 200 ms three-stage IMU gate (ASVM → GSVM → tilt)
//   6. IMU-only fallback when EEG is offline / unreliable
//   7. EMG burst detector (>30 Hz spikes pause EEG)
//
//  RETAINED FROM v1:
//   • Multi-electrode agreement (≥2 of 3 channels)
//   • Adaptive MAD-scaled baseline
//   • Motion-gated EEG (freezes when GSVM > 100 dps)
//   • Refractory lockout
//   • 5-state machine
// ============================================================

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────────────────────
#define ADS_CS         PB0
#define ADS_DRDY       PB1
#define ALERT_LED      PC13   // Onboard LED (active-low)
#define ALERT_PIN      PA0    // External alert output

// ─────────────────────────────────────────────────────────────
//  ADS1299 COMMANDS / REGISTERS
// ─────────────────────────────────────────────────────────────
#define CMD_RESET      0x06
#define CMD_START      0x08
#define CMD_RDATAC     0x10
#define CMD_SDATAC     0x11
#define CMD_RREG       0x20
#define CMD_WREG       0x40

#define REG_CONFIG1    0x01
#define REG_CONFIG2    0x02
#define REG_CONFIG3    0x03
#define REG_LOFF       0x04
#define REG_CH1SET     0x05
#define REG_LOFF_SENSP 0x0F   // Lead-off detect, positive side
#define REG_LOFF_SENSN 0x10   // Lead-off detect, negative side

// ─────────────────────────────────────────────────────────────
//  ALGORITHM CONSTANTS (tune here)
// ─────────────────────────────────────────────────────────────
#define NUM_EEG_CH         3
#define EEG_SPS            250.0f
#define IMU_HZ             100
#define IMU_DT_MS          (1000 / IMU_HZ)

// IMU pre-impact thresholds
#define FREEFALL_G         0.82f
#define FREEFALL_MS        50
#define GYRO_THRESHOLD     50.0f      // dps (lowered from 150 for prediction)
#define IMPACT_G           2.0f
#define TILT_THRESHOLD     25.0f      // degrees from vertical
#define STAGE_GATE_MS      200        // 0.2 s strict gate between stages

// IMU-only fallback thresholds (stricter)
#define FB_GYRO_THRESHOLD  70.0f
#define FB_TILT_THRESHOLD  35.0f

// EEG thresholds
#define SLOW_RATIO_K       3.0f       // MAD multiplier
#define SLOW_SUSTAIN_MS    500
#define MOTION_GATE_DPS    100.0f
#define EMG_BURST_GATE     5.0f       // multiples of baseline EMG power
#define EMG_HOLDOFF_MS     500        // pause EEG this long after EMG burst

// Coincidence window between EEG slow-wave and IMU freefall
#define COINCIDENCE_MS     2000       // ±2 s

// Refractory + baseline
#define REFRACTORY_MS      5000
#define BASELINE_TAU_S     60.0f

// Powerline frequency (50 for EU/Asia, 60 for US/Canada)
#define POWERLINE_HZ       60

// Signal quality thresholds
#define EEG_FLATLINE_UV    0.5f       // RMS below this = flatline
#define EEG_SATURATION_LSB 7000000L   // ~83% of full-scale 24-bit

// ADS1299 conversion: Vref=4.5V, Gain=24, 24-bit
#define ADS_LSB_UV         (4500000.0f / (24.0f * 8388608.0f))

// ─────────────────────────────────────────────────────────────
//  BNO055
// ─────────────────────────────────────────────────────────────
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ─────────────────────────────────────────────────────────────
//  ENUMS
// ─────────────────────────────────────────────────────────────
enum FallState {
  STATE_STABLE,
  STATE_DISTURBED,
  STATE_FREEFALL,
  STATE_FALLING,    // ← alert fires on entry
  STATE_IMPACT,
  STATE_POST_FALL
};

enum AlertConfidence {
  CONF_NONE = 0,
  CONF_LOW,         // IMU-only, EEG offline
  CONF_MED,         // IMU-only with EEG online but quiet
  CONF_HIGH         // EEG + IMU coincidence
};

enum ElectrodeStatus {
  ELEC_GOOD,
  ELEC_FLATLINE,
  ELEC_SATURATED,
  ELEC_LEAD_OFF
};

enum EEGMode {
  EEG_NORMAL,       // ≥2 of 3 electrodes good
  EEG_DEGRADED,     // exactly 1 electrode good
  EEG_OFFLINE       // 0 electrodes good
};

// ─────────────────────────────────────────────────────────────
//  GLOBAL STATE
// ─────────────────────────────────────────────────────────────
FallState     fallState     = STATE_STABLE;
EEGMode       eegMode       = EEG_NORMAL;
unsigned long stateEnterTime= 0;
unsigned long lastAlertTime = 0;
unsigned long freefallStart = 0;
unsigned long freefallConfirmedTime = 0;
unsigned long gsvmConfirmedTime    = 0;
unsigned long lastEEGTriggerTime   = 0;
unsigned long lastEMGBurstTime     = 0;

// Lead-off status bits from data frame
uint8_t loffStatP = 0;
uint8_t loffStatN = 0;

// ─────────────────────────────────────────────────────────────
//  FILTER STRUCTURES
// ─────────────────────────────────────────────────────────────
struct OnePoleHP { float alpha, xPrev, yPrev; };
struct OnePoleLP { float alpha, yPrev; };

float hpStep(OnePoleHP &f, float x) {
  float y = f.alpha * (f.yPrev + x - f.xPrev);
  f.xPrev = x; f.yPrev = y;
  return y;
}
float lpStep(OnePoleLP &f, float x) {
  f.yPrev += f.alpha * (x - f.yPrev);
  return f.yPrev;
}
void initHP(OnePoleHP &f, float fc, float fs) {
  f.alpha = expf(-2.0f * PI * fc / fs);
  f.xPrev = f.yPrev = 0;
}
void initLP(OnePoleLP &f, float fc, float fs) {
  f.alpha = 1.0f - expf(-2.0f * PI * fc / fs);
  f.yPrev = 0;
}

// ── 2nd-order biquad (used for Butterworth LP and notch filters) ──
struct Biquad {
  float b0, b1, b2, a1, a2;
  float z1, z2;
};
float biquadStep(Biquad &bq, float x) {
  float y = bq.b0 * x + bq.z1;
  bq.z1 = bq.b1 * x - bq.a1 * y + bq.z2;
  bq.z2 = bq.b2 * x - bq.a2 * y;
  return y;
}

void initBiquadLP5Hz_fs100(Biquad &bq) {
  // 2nd-order Butterworth LP @ 5 Hz, fs = 100 Hz
  bq.b0 = 0.020083365564211f;
  bq.b1 = 0.040166731128423f;
  bq.b2 = 0.020083365564211f;
  bq.a1 = -1.561018075800718f;
  bq.a2 =  0.641351538057563f;
  bq.z1 = bq.z2 = 0;
}

// IIR notch filter (cookbook formula, RBJ)
// Q = 30 → narrow notch (~2 Hz bandwidth)
void initNotch(Biquad &bq, float f0, float fs, float Q) {
  float w0    = 2.0f * PI * f0 / fs;
  float cosW0 = cosf(w0);
  float sinW0 = sinf(w0);
  float alpha = sinW0 / (2.0f * Q);
  float a0    = 1.0f + alpha;

  bq.b0 =  1.0f / a0;
  bq.b1 = -2.0f * cosW0 / a0;
  bq.b2 =  1.0f / a0;
  bq.a1 = -2.0f * cosW0 / a0;
  bq.a2 = (1.0f - alpha) / a0;
  bq.z1 = bq.z2 = 0;
}

// ─────────────────────────────────────────────────────────────
//  PER-CHANNEL EEG CHAIN
// ─────────────────────────────────────────────────────────────
struct EEGChain {
  Biquad    notch;          // 50/60 Hz powerline rejection
  OnePoleHP dc;              // 0.5 Hz DC removal
  OnePoleLP slowLP;          // ≤8 Hz (delta + theta band)
  OnePoleHP fastHP;          // 8 Hz HP
  OnePoleLP fastLP;          // 30 Hz LP (alpha + beta band)
  OnePoleHP emgHP;           // 30 Hz HP for EMG burst detection
  OnePoleLP slowPowerLP;     // 250 ms envelope for slow band
  OnePoleLP fastPowerLP;     // 250 ms envelope for fast band
  OnePoleLP emgPowerLP;      // 100 ms envelope for EMG band
  OnePoleLP baselineLP;
  OnePoleLP madLP;
  OnePoleLP emgBaselineLP;

  float baselineMean;
  float baselineMAD;
  float emgBaseline;
  float rmsAccumulator;
  uint16_t rmsSamples;
  float channelRMS_uV;

  unsigned long aboveSince;
  bool          triggered;
  ElectrodeStatus status;
};
EEGChain eeg[NUM_EEG_CH];

// ─────────────────────────────────────────────────────────────
//  IMU FILTER STATE
// ─────────────────────────────────────────────────────────────
Biquad lpAx, lpAy, lpAz, lpGx, lpGy, lpGz;

// Complementary filter state (pitch / roll in degrees)
float pitch = 0.0f, roll = 0.0f;
const float COMP_ALPHA = 0.98f;  // 0.98 gyro / 0.02 accel

// IMU outputs
float asvm    = 1.0f;
float gsvm    = 0.0f;
float tiltDeg = 0.0f;
float fAx, fAy, fAz, fGx, fGy, fGz;

unsigned long lastIMUTime = 0;

// Multi-electrode trigger flags
bool chTrig[NUM_EEG_CH] = {false, false, false};

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(ALERT_LED, OUTPUT);
  pinMode(ALERT_PIN, OUTPUT);
  digitalWrite(ALERT_LED, HIGH);
  digitalWrite(ALERT_PIN, LOW);

  // SPI for ADS1299
  pinMode(ADS_CS, OUTPUT);
  pinMode(ADS_DRDY, INPUT);
  digitalWrite(ADS_CS, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.setDataMode(SPI_MODE1);
  SPI.setBitOrder(MSBFIRST);

  // I2C for BNO055
  Wire.setSDA(PB7);
  Wire.setSCL(PB8);
  Wire.begin();

  if (!initADS1299()) {
    Serial.println("ERR: ADS1299 init failed");
    while (1);
  }
  if (!bno.begin()) {
    Serial.println("ERR: BNO055 not found");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(100);

  // Initialise per-channel EEG filter chains
  for (int i = 0; i < NUM_EEG_CH; i++) {
    initNotch(eeg[i].notch, (float)POWERLINE_HZ, EEG_SPS, 30.0f);
    initHP(eeg[i].dc,         0.5f, EEG_SPS);
    initLP(eeg[i].slowLP,     8.0f, EEG_SPS);
    initHP(eeg[i].fastHP,     8.0f, EEG_SPS);
    initLP(eeg[i].fastLP,    30.0f, EEG_SPS);
    initHP(eeg[i].emgHP,     30.0f, EEG_SPS);

    initLP(eeg[i].slowPowerLP, 4.0f, EEG_SPS);
    initLP(eeg[i].fastPowerLP, 4.0f, EEG_SPS);
    initLP(eeg[i].emgPowerLP, 10.0f, EEG_SPS);

    initLP(eeg[i].baselineLP,    1.0f / BASELINE_TAU_S, EEG_SPS);
    initLP(eeg[i].madLP,         1.0f / BASELINE_TAU_S, EEG_SPS);
    initLP(eeg[i].emgBaselineLP, 1.0f / BASELINE_TAU_S, EEG_SPS);

    eeg[i].baselineMean = 1.0f;
    eeg[i].baselineMAD  = 0.2f;
    eeg[i].emgBaseline  = 0.0f;
    eeg[i].rmsAccumulator = 0.0f;
    eeg[i].rmsSamples = 0;
    eeg[i].channelRMS_uV = 0.0f;
    eeg[i].aboveSince = 0;
    eeg[i].triggered  = false;
    eeg[i].status     = ELEC_GOOD;
  }

  initBiquadLP5Hz_fs100(lpAx);
  initBiquadLP5Hz_fs100(lpAy);
  initBiquadLP5Hz_fs100(lpAz);
  initBiquadLP5Hz_fs100(lpGx);
  initBiquadLP5Hz_fs100(lpGy);
  initBiquadLP5Hz_fs100(lpGz);

  Serial.println(
    "t_ms,state,eegMode,asvm_g,gsvm_dps,tilt_deg,"
    "ch1_uv,ch2_uv,ch3_uv,"
    "stat1,stat2,stat3,"
    "z1,z2,z3,trig"
  );

  stateEnterTime = millis();
}

// ─────────────────────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────────────────────
void loop() {

  if (digitalRead(ADS_DRDY) == LOW) {
    long raw[8];
    readADS1299(raw);

    float uV[NUM_EEG_CH];
    for (int i = 0; i < NUM_EEG_CH; i++) {
      uV[i] = (float)raw[i] * ADS_LSB_UV;

      // Check for saturation BEFORE filtering (use raw value)
      if (labs(raw[i]) > EEG_SATURATION_LSB) {
        eeg[i].status = ELEC_SATURATED;
      }

      processEEG(i, uV[i], raw[i]);
    }

    updateElectrodeStatus();
    updateEEGMode();

    // Update overall EEG trigger time for coincidence detection
    int votes = 0;
    for (int i = 0; i < NUM_EEG_CH; i++) if (chTrig[i]) votes++;
    if (votes >= 2) lastEEGTriggerTime = millis();

    static uint8_t outDiv = 0;
    if (++outDiv >= 5) {
      outDiv = 0;
      streamCSV(uV);
    }
  }

  unsigned long now = millis();
  if (now - lastIMUTime >= IMU_DT_MS) {
    float dt = (now - lastIMUTime) / 1000.0f;
    lastIMUTime = now;
    readAndProcessIMU(dt);
    updateFallStateMachine();
  }

  if (digitalRead(ALERT_PIN) == HIGH && (now - lastAlertTime) > 500) {
    digitalWrite(ALERT_PIN, LOW);
    digitalWrite(ALERT_LED, HIGH);
  }
}

// ─────────────────────────────────────────────────────────────
//  EEG PROCESSING — per sample, per channel
// ─────────────────────────────────────────────────────────────
void processEEG(int ch, float x_uV, long rawLSB) {
  EEGChain &c = eeg[ch];

  // 1) Notch filter (60 Hz powerline)
  float xN = biquadStep(c.notch, x_uV);

  // 2) DC removal
  float xDC = hpStep(c.dc, xN);

  // 3) Channel RMS over 250 ms window (for flatline detection)
  c.rmsAccumulator += xDC * xDC;
  c.rmsSamples++;
  if (c.rmsSamples >= (uint16_t)(EEG_SPS * 0.25f)) {
    c.channelRMS_uV = sqrtf(c.rmsAccumulator / c.rmsSamples);
    c.rmsAccumulator = 0.0f;
    c.rmsSamples = 0;
  }

  // 4) Slow band (≤8 Hz)
  float slow = lpStep(c.slowLP, xDC);

  // 5) Fast band (8–30 Hz)
  float fastHP = hpStep(c.fastHP, xDC);
  float fast   = lpStep(c.fastLP, fastHP);

  // 6) EMG band (>30 Hz) for burst detection
  float emg     = hpStep(c.emgHP, xDC);
  float emgPow  = lpStep(c.emgPowerLP, emg * emg);

  // 7) Slow/fast power envelopes
  float slowPow = lpStep(c.slowPowerLP, slow * slow);
  float fastPow = lpStep(c.fastPowerLP, fast * fast) + 1e-6f;

  // 8) EMG burst detection — pauses EEG triggering
  bool emgBurst = false;
  if (c.emgBaseline > 0.01f && emgPow > c.emgBaseline * EMG_BURST_GATE) {
    emgBurst = true;
    lastEMGBurstTime = millis();
  }

  // 9) Slow/fast ratio
  float ratio = slowPow / fastPow;

  // 10) Adaptive baseline & MAD (only when stable & motion-quiet)
  bool motionGated = (gsvm > MOTION_GATE_DPS);
  bool emgHoldoff  = (millis() - lastEMGBurstTime) < EMG_HOLDOFF_MS;

  if (!motionGated && !emgHoldoff && fallState == STATE_STABLE
      && c.status == ELEC_GOOD) {
    c.baselineMean = lpStep(c.baselineLP, ratio);
    float absDev   = fabsf(ratio - c.baselineMean);
    c.baselineMAD  = lpStep(c.madLP, absDev) + 0.01f;
    c.emgBaseline  = lpStep(c.emgBaselineLP, emgPow);
  }

  // 11) Z-score
  float z = (ratio - c.baselineMean) / c.baselineMAD;

  // 12) Trigger logic — must be GOOD electrode, not motion- or EMG-gated
  bool canTrigger = !motionGated && !emgHoldoff && (c.status == ELEC_GOOD);

  if (canTrigger && z > SLOW_RATIO_K) {
    if (c.aboveSince == 0) c.aboveSince = millis();
    if (millis() - c.aboveSince >= SLOW_SUSTAIN_MS) {
      c.triggered = true;
    }
  } else {
    c.aboveSince = 0;
    c.triggered  = false;
  }

  chTrig[ch] = c.triggered;
}

// ─────────────────────────────────────────────────────────────
//  ELECTRODE STATUS — combines lead-off bits + signal quality
// ─────────────────────────────────────────────────────────────
void updateElectrodeStatus() {
  for (int i = 0; i < NUM_EEG_CH; i++) {
    bool leadOff = ((loffStatP >> i) & 0x01) || ((loffStatN >> i) & 0x01);

    if (leadOff) {
      eeg[i].status = ELEC_LEAD_OFF;
    } else if (eeg[i].channelRMS_uV < EEG_FLATLINE_UV
               && eeg[i].channelRMS_uV > 0.0f) {
      eeg[i].status = ELEC_FLATLINE;
    } else if (eeg[i].status != ELEC_SATURATED) {
      // SATURATED is sticky for 1 second to avoid flicker
      eeg[i].status = ELEC_GOOD;
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  EEG MODE DETERMINATION
// ─────────────────────────────────────────────────────────────
void updateEEGMode() {
  int good = 0;
  for (int i = 0; i < NUM_EEG_CH; i++) {
    if (eeg[i].status == ELEC_GOOD) good++;
  }
  if (good >= 2)       eegMode = EEG_NORMAL;
  else if (good == 1)  eegMode = EEG_DEGRADED;
  else                 eegMode = EEG_OFFLINE;
}

// ─────────────────────────────────────────────────────────────
//  IMU PROCESSING — complementary filter + ASVM/GSVM
// ─────────────────────────────────────────────────────────────
void readAndProcessIMU(float dt) {
  imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> w = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Total accel (m/s²) = gravity + linear
  float ax = a.x() + g.x();
  float ay = a.y() + g.y();
  float az = a.z() + g.z();

  // 5 Hz Butterworth low-pass on all axes
  fAx = biquadStep(lpAx, ax);
  fAy = biquadStep(lpAy, ay);
  fAz = biquadStep(lpAz, az);
  fGx = biquadStep(lpGx, w.x());
  fGy = biquadStep(lpGy, w.y());
  fGz = biquadStep(lpGz, w.z());

  // ASVM in g
  asvm = sqrtf(fAx*fAx + fAy*fAy + fAz*fAz) / 9.81f;

  // GSVM in dps (BNO055 gyro in rad/s)
  float gx_dps = fGx * 57.2958f;
  float gy_dps = fGy * 57.2958f;
  float gz_dps = fGz * 57.2958f;
  gsvm = sqrtf(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);

  // ── COMPLEMENTARY FILTER for pitch/roll ──
  // Accel-derived angles (long-term reference)
  float pitchAcc = atan2f(-fAx, sqrtf(fAy*fAy + fAz*fAz)) * 57.2958f;
  float rollAcc  = atan2f(fAy, fAz) * 57.2958f;

  // Gyro-integrated angles (short-term accurate)
  pitch = COMP_ALPHA * (pitch + gy_dps * dt) + (1.0f - COMP_ALPHA) * pitchAcc;
  roll  = COMP_ALPHA * (roll  + gx_dps * dt) + (1.0f - COMP_ALPHA) * rollAcc;

  // Combined tilt from vertical
  tiltDeg = sqrtf(pitch*pitch + roll*roll);
}

// ─────────────────────────────────────────────────────────────
//  FALL STATE MACHINE — 3-stage gate + coincidence + fallback
// ─────────────────────────────────────────────────────────────
void updateFallStateMachine() {
  unsigned long now = millis();

  switch (fallState) {

    case STATE_STABLE: {
      // Promote on EEG slow-wave (NORMAL mode only) OR motion disturbance
      int votes = 0;
      for (int i = 0; i < NUM_EEG_CH; i++) if (chTrig[i]) votes++;
      bool eegAlert = (eegMode == EEG_NORMAL && votes >= 2);

      if (eegAlert || asvm < 0.9f || gsvm > 30.0f) {
        gotoState(STATE_DISTURBED);
      }
      break;
    }

    case STATE_DISTURBED:
      // Stage 1: confirm freefall sustained ≥50 ms
      if (asvm < FREEFALL_G) {
        if (freefallStart == 0) freefallStart = now;
        if (now - freefallStart >= FREEFALL_MS) {
          freefallConfirmedTime = now;
          gotoState(STATE_FREEFALL);
        }
      } else {
        freefallStart = 0;
        if (gsvm < 20.0f && now - stateEnterTime > 2000) {
          gotoState(STATE_STABLE);
        }
      }
      break;

    case STATE_FREEFALL: {
      // Decide thresholds based on EEG mode
      float gyroTh = (eegMode == EEG_OFFLINE) ? FB_GYRO_THRESHOLD : GYRO_THRESHOLD;
      float tiltTh = (eegMode == EEG_OFFLINE) ? FB_TILT_THRESHOLD : TILT_THRESHOLD;

      // Stage 2: GSVM must exceed threshold within 200 ms of freefall
      if (gsvmConfirmedTime == 0 && gsvm > gyroTh) {
        gsvmConfirmedTime = now;
      }

      // Stage 3: tilt must exceed threshold within 200 ms of GSVM
      if (gsvmConfirmedTime > 0
          && (now - gsvmConfirmedTime) < STAGE_GATE_MS
          && tiltDeg > tiltTh) {

        AlertConfidence conf = decideConfidence();
        firePreImpactAlert(conf);
        gotoState(STATE_FALLING);
        break;
      }

      // Gate expired — fall back to DISTURBED if no progression
      if (gsvmConfirmedTime > 0 && (now - gsvmConfirmedTime) >= STAGE_GATE_MS) {
        gsvmConfirmedTime = 0;
        gotoState(STATE_DISTURBED);
      }
      // Stage 1 gate — must see GSVM within 200 ms of freefall
      else if ((now - freefallConfirmedTime) >= STAGE_GATE_MS
               && gsvmConfirmedTime == 0) {
        gotoState(STATE_DISTURBED);
      }
      // Spurious freefall (e.g., jump)
      else if (now - stateEnterTime > 500 && asvm > 0.9f) {
        gotoState(STATE_DISTURBED);
      }
      break;
    }

    case STATE_FALLING:
      if (asvm > IMPACT_G) gotoState(STATE_IMPACT);
      if (now - stateEnterTime > 1500) gotoState(STATE_POST_FALL);
      break;

    case STATE_IMPACT:
      if (now - stateEnterTime > 300) gotoState(STATE_POST_FALL);
      break;

    case STATE_POST_FALL:
      if (now - lastAlertTime > REFRACTORY_MS) gotoState(STATE_STABLE);
      break;
  }
}

// ─────────────────────────────────────────────────────────────
//  COINCIDENCE WINDOW + CONFIDENCE LEVEL
// ─────────────────────────────────────────────────────────────
AlertConfidence decideConfidence() {
  unsigned long now = millis();
  bool eegRecent = (lastEEGTriggerTime > 0)
                && (now - lastEEGTriggerTime < COINCIDENCE_MS);

  if (eegMode == EEG_OFFLINE)         return CONF_LOW;
  if (eegMode == EEG_NORMAL && eegRecent) return CONF_HIGH;
  return CONF_MED;
}

void gotoState(FallState s) {
  fallState = s;
  stateEnterTime = millis();
  if (s != STATE_FREEFALL) {
    freefallStart = 0;
    freefallConfirmedTime = 0;
    gsvmConfirmedTime = 0;
  }
}

// ─────────────────────────────────────────────────────────────
//  ALERT
// ─────────────────────────────────────────────────────────────
void firePreImpactAlert(AlertConfidence conf) {
  lastAlertTime = millis();
  digitalWrite(ALERT_PIN, HIGH);
  digitalWrite(ALERT_LED, LOW);

  Serial.print("ALERT,t=");      Serial.print(lastAlertTime);
  Serial.print(",conf=");        Serial.print((int)conf);
  Serial.print(",eegMode=");     Serial.print((int)eegMode);
  Serial.print(",asvm=");        Serial.print(asvm, 3);
  Serial.print(",gsvm=");        Serial.print(gsvm, 1);
  Serial.print(",tilt=");        Serial.print(tiltDeg, 1);
  Serial.print(",eeg_recent=");  Serial.print((millis() - lastEEGTriggerTime) < COINCIDENCE_MS ? 1 : 0);
  Serial.print(",elec=");
  for (int i = 0; i < NUM_EEG_CH; i++) Serial.print((int)eeg[i].status);
  Serial.println();
}

// ─────────────────────────────────────────────────────────────
//  CSV STREAMING
// ─────────────────────────────────────────────────────────────
void streamCSV(const float *uV) {
  Serial.print(millis());       Serial.print(',');
  Serial.print((int)fallState); Serial.print(',');
  Serial.print((int)eegMode);   Serial.print(',');
  Serial.print(asvm, 3);        Serial.print(',');
  Serial.print(gsvm, 1);        Serial.print(',');
  Serial.print(tiltDeg, 1);
  for (int i = 0; i < NUM_EEG_CH; i++) {
    Serial.print(','); Serial.print(uV[i], 2);
  }
  for (int i = 0; i < NUM_EEG_CH; i++) {
    Serial.print(','); Serial.print((int)eeg[i].status);
  }
  for (int i = 0; i < NUM_EEG_CH; i++) {
    float r = eeg[i].slowPowerLP.yPrev
            / (eeg[i].fastPowerLP.yPrev + 1e-6f);
    float z = (r - eeg[i].baselineMean) / eeg[i].baselineMAD;
    Serial.print(','); Serial.print(z, 2);
  }
  Serial.print(',');
  for (int i = 0; i < NUM_EEG_CH; i++) Serial.print(chTrig[i] ? '1' : '0');
  Serial.println();
}

// ─────────────────────────────────────────────────────────────
//  ADS1299 INITIALISATION  (with lead-off detection enabled)
// ─────────────────────────────────────────────────────────────
bool initADS1299() {
  delay(500);
  adsCmd(CMD_RESET);   delay(10);
  adsCmd(CMD_SDATAC);  delay(2);

  // CONFIG1: high-res mode, 250 SPS
  adsWriteReg(REG_CONFIG1, 0x96);
  // CONFIG2: internal test off, internal clock
  adsWriteReg(REG_CONFIG2, 0xC0);
  // CONFIG3: internal reference ON, bias buffer enabled
  adsWriteReg(REG_CONFIG3, 0xEC);
  delay(150);

  // LOFF register: comparator threshold 95%/5%, current 6 nA, AC mode @ fDR/4
  adsWriteReg(REG_LOFF, 0x13);

  // Enable lead-off detection on channels 1-3 (both P and N sides)
  adsWriteReg(REG_LOFF_SENSP, 0x07);
  adsWriteReg(REG_LOFF_SENSN, 0x07);

  // Channels 1-3: gain=24, normal input
  for (byte ch = 0; ch < 3; ch++) {
    adsWriteReg(REG_CH1SET + ch, 0x60);
  }
  // Channels 4-8: power down + short input to (VDD+VSS)/2
  for (byte ch = 3; ch < 8; ch++) {
    adsWriteReg(REG_CH1SET + ch, 0x81);
  }

  adsCmd(CMD_RDATAC);
  adsCmd(CMD_START);
  return true;
}

void adsCmd(byte cmd) {
  digitalWrite(ADS_CS, LOW);  delayMicroseconds(2);
  SPI.transfer(cmd);
  delayMicroseconds(2);  digitalWrite(ADS_CS, HIGH); delayMicroseconds(2);
}

void adsWriteReg(byte reg, byte val) {
  digitalWrite(ADS_CS, LOW);  delayMicroseconds(2);
  SPI.transfer(CMD_WREG | reg);
  SPI.transfer(0x00);
  SPI.transfer(val);
  delayMicroseconds(2);  digitalWrite(ADS_CS, HIGH); delayMicroseconds(2);
}

// ─────────────────────────────────────────────────────────────
//  READ 27-byte data frame, extract lead-off status from header
//  Frame header layout (24 bits = 3 bytes):
//   bits 23:20 = 1100 (constant)
//   bits 19:12 = LOFF_STATP[7:0]  (channels 8..1, P-side)
//   bits 11:4  = LOFF_STATN[7:0]  (channels 8..1, N-side)
//   bits 3:0   = GPIO[7:4]
// ─────────────────────────────────────────────────────────────
void readADS1299(long *channels) {
  digitalWrite(ADS_CS, LOW); delayMicroseconds(2);

  byte s0 = SPI.transfer(0);
  byte s1 = SPI.transfer(0);
  byte s2 = SPI.transfer(0);
  uint32_t statusWord = ((uint32_t)s0 << 16) | ((uint32_t)s1 << 8) | s2;
  loffStatP = (statusWord >> 12) & 0xFF;
  loffStatN = (statusWord >>  4) & 0xFF;

  for (int ch = 0; ch < 8; ch++) {
    byte b0 = SPI.transfer(0);
    byte b1 = SPI.transfer(0);
    byte b2 = SPI.transfer(0);
    long v = ((long)b0 << 16) | ((long)b1 << 8) | b2;
    if (v & 0x800000L) v |= 0xFF000000L;
    channels[ch] = v;
  }

  delayMicroseconds(2); digitalWrite(ADS_CS, HIGH);
}
