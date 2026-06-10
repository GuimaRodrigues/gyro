#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>

/*
  Firmware simples para gyro de drift RC RWD 1/10 com Arduino Uno/Nano e MPU6050.

  Hardware:
  - MPU6050 via I2C: SDA=A4, SCL=A5
  - CH1 steering do receptor: D2 / INT0
  - CH5 gyro gain: D3 / INT1
  - CH6 damper / anti-hunting: D4 / Pin Change Interrupt
  - CH4 botao de recalibracao: D7 / Pin Change Interrupt
  - Servo de direcao: D9, sinal gerado por Timer1 em 50 Hz
  - LED onboard: LED_BUILTIN / D13 para feedback visual de recalibracao

  Ajustes principais:
  - CH5 ajusta o ganho do gyro. Abaixo de GYRO_GAIN_OFF_THRESHOLD_US o gyro fica
    desligado, mas os endpoints fisicos continuam sendo aplicados.
  - CH6 ajusta o damper. Valores baixos deixam a resposta mais rapida; valores
    altos aumentam smoothing/deadband para reduzir hunting e tremedeira.
  - SERVO_LEFT_LIMIT_US e SERVO_RIGHT_LIMIT_US sao limites fisicos finais. Eles
    protegem o servo depois da soma steering + gyro.
  - CH4 recalibra com long press: segure por RECAL_HOLD_MS com o carro parado.
    Clique curto nao faz nada. Uma nova recalibracao so e armada apos soltar CH4.

  Teste rapido de direcao:
  - Levante o carro, vire a frente para a esquerda/direita e confirme que o gyro
    corrige no sentido de contra-esterco.
  - Se corrigir ao contrario, altere GYRO_REVERSE.
  - Se o comando do radio estiver invertido, altere SERVO_REVERSE.
*/

#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega168__)
#error "Este firmware usa Timer1 do Arduino Uno/Nano classico com ATmega328P/ATmega168."
#endif

// ------------------------- Pinos -------------------------
const uint8_t RX_STEERING_PIN = 2;     // CH1 steering: interrupcao externa INT0
const uint8_t RX_GYRO_GAIN_PIN = 3;    // CH5 gyro gain: interrupcao externa INT1
const uint8_t RX_DAMPER_PIN = 4;       // CH6 damper / anti-hunting: PCINT20
const uint8_t RX_RECAL_PIN = 7;        // CH4 botao de recalibracao: PCINT23
const uint8_t SERVO_OUTPUT_PIN = 9;    // Saida do servo gerada por Timer1

// ------------------------- Servo / receptor -------------------------
const uint16_t RX_COMMAND_MIN_US = 1000;
const uint16_t RX_COMMAND_CENTER_US = 1500;
const uint16_t RX_COMMAND_MAX_US = 2000;
const uint16_t RX_VALID_MIN_US = 900;
const uint16_t RX_VALID_MAX_US = 2100;
const uint32_t RX_SIGNAL_TIMEOUT_US = 100000UL;

const uint16_t SERVO_CENTER_US = 1500;
const uint16_t SERVO_LEFT_LIMIT_US = 1270;
const uint16_t SERVO_RIGHT_LIMIT_US = 1730;
const uint16_t SERVO_FRAME_US = 20000;  // 50 Hz

// Inverta se o servo estiver respondendo ao contrario do radio.
const bool SERVO_REVERSE = false;

// ------------------------- Gyro / controle -------------------------
const uint8_t MPU6050_ADDRESS = 0x68;
const uint32_t I2C_CLOCK_HZ = 400000UL;
const uint8_t MPU6050_DLPF_CFG = 2;         // 2 = filtro interno ~94 Hz, baixa latencia
const float GYRO_LSB_PER_DPS = 65.5f;       // FS_SEL=1, faixa +/-500 graus/s
const uint16_t GYRO_CALIBRATION_SAMPLES = 1500;
const uint16_t GYRO_CALIBRATION_INTERVAL_US = 1000;
const uint16_t CONTROL_INTERVAL_US = 1000;  // controle a 1 kHz

// Inverta se o gyro corrigir para o lado errado.
const bool GYRO_REVERSE = false;

const float GYRO_GAIN_MIN = 0.0f;
const float GYRO_GAIN_MAX = 3.0f;
const uint16_t GYRO_GAIN_OFF_THRESHOLD_US = 1080;
const float GYRO_GAIN_FAILSAFE = 1.5f;

const float DAMPER_SMOOTHING_MIN = 0.60f;
const float DAMPER_SMOOTHING_MAX = 0.80f;
const float DAMPER_DEADBAND_MIN_DPS = 1.5f;
const float DAMPER_DEADBAND_MAX_DPS = 3.0f;
const float DAMPER_FAILSAFE_SMOOTHING = 0.70f;
const float DAMPER_FAILSAFE_DEADBAND_DPS = 2.0f;

const float ADAPTIVE_DAMPER_LOW_DPS = 8.0f;
const float ADAPTIVE_DAMPER_HIGH_DPS = 80.0f;
const float ADAPTIVE_SMOOTHING_BOOST_AT_LOW_RATE = 0.06f;
const float ADAPTIVE_SMOOTHING_REDUCTION_AT_HIGH_RATE = 0.05f;
const float ADAPTIVE_DEADBAND_BOOST_AT_LOW_RATE_DPS = 0.6f;
const float ADAPTIVE_DEADBAND_REDUCTION_AT_HIGH_RATE_DPS = 0.4f;

const float PROGRESSIVE_GAIN_START_DPS = 20.0f;
const float PROGRESSIVE_GAIN_FULL_DPS = 120.0f;
const float PROGRESSIVE_GAIN_MIN_FACTOR = 0.75f;
const float PROGRESSIVE_GAIN_MAX_FACTOR = 1.18f;

const int16_t GYRO_CORRECTION_LIMIT_US = 250;
const uint16_t DRIVER_PRIORITY_FULL_STEER_US = 230;
const float DRIVER_PRIORITY_MIN_FACTOR = 0.55f;

// ------------------------- Recalibracao CH4 -------------------------
const uint16_t RECAL_BUTTON_ACTIVE_US = 1700;
const uint16_t RECAL_HOLD_MS = 2500;
const float RECAL_MAX_ABS_GYRO_DPS = 5.0f;
const uint16_t RECAL_SAMPLE_INTERVAL_US = 1000;
const uint16_t RECAL_STABILITY_SAMPLES = 80;
const uint16_t RECAL_OFFSET_SAMPLES = 300;

// ------------------------- LED de status -------------------------
const uint16_t STATUS_LED_FAST_ON_MS = 80;
const uint16_t STATUS_LED_FAST_OFF_MS = 80;
const uint16_t STATUS_LED_LONG_ON_MS = 350;
const uint16_t STATUS_LED_LONG_OFF_MS = 250;

// ------------------------- Debug -------------------------
const bool DEBUG_SERIAL = false;
const uint32_t DEBUG_BAUD = 115200;
const uint16_t DEBUG_INTERVAL_MS = 100;

// ------------------------- Registradores do MPU6050 -------------------------
const uint8_t MPU_REG_SMPLRT_DIV = 0x19;
const uint8_t MPU_REG_CONFIG = 0x1A;
const uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
const uint8_t MPU_REG_GYRO_ZOUT_H = 0x47;
const uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;

// Timer1 em Uno/Nano: 16 MHz / prescaler 8 = 2 ticks por microssegundo.
const uint16_t TIMER1_TICKS_PER_US = 2;
const uint16_t SERVO_FRAME_TICKS = SERVO_FRAME_US * TIMER1_TICKS_PER_US;

const uint8_t RX_DAMPER_MASK = _BV(PD4);
const uint8_t RX_RECAL_MASK = _BV(PD7);

struct PulseSnapshot {
  uint16_t pulseUs;
  uint32_t lastPulseUs;
  bool hasPulse;
};

struct RxChannel {
  volatile uint32_t riseUs;
  volatile uint16_t pulseUs;
  volatile uint32_t lastPulseUs;
  volatile bool hasPulse;
};

enum RecalState {
  RECAL_IDLE,
  RECAL_STABILITY_CHECK,
  RECAL_SAMPLING_OFFSET,
  RECAL_WAIT_RELEASE
};

volatile RxChannel steeringChannel = {0, RX_COMMAND_CENTER_US, 0, false};
volatile RxChannel gainChannel = {0, RX_COMMAND_CENTER_US, 0, false};
volatile RxChannel damperChannel = {0, RX_COMMAND_CENTER_US, 0, false};
volatile RxChannel recalChannel = {0, RX_COMMAND_CENTER_US, 0, false};
volatile uint8_t lastPortDState = 0;

volatile uint16_t servoPulseTicks = SERVO_CENTER_US * TIMER1_TICKS_PER_US;
volatile uint8_t *servoPort = 0;
uint8_t servoBitMask = 0;

bool mpuReady = false;
float gyroZOffsetRaw = 0.0f;
float filteredCorrectionUs = 0.0f;
float activeGyroGain = GYRO_GAIN_FAILSAFE;
float activeSmoothingAlpha = DAMPER_FAILSAFE_SMOOTHING;
float activeGyroDeadbandDps = DAMPER_FAILSAFE_DEADBAND_DPS;
bool activeGyroEnabled = true;
uint16_t steeringNeutralUs = RX_COMMAND_CENTER_US;
uint32_t lastControlUs = 0;

RecalState recalState = RECAL_IDLE;
uint32_t recalPressStartMs = 0;
uint32_t recalNextSampleUs = 0;
uint16_t recalAttemptSamples = 0;
uint16_t recalGoodSamples = 0;
int32_t recalRawSum = 0;
uint16_t recalPendingNeutralUs = RX_COMMAND_CENTER_US;
bool recalButtonActive = false;

bool statusLedBlinkActive = false;
bool statusLedIsOn = false;
uint8_t statusLedTargetBlinks = 0;
uint8_t statusLedCompletedBlinks = 0;
uint16_t statusLedOnMs = 0;
uint16_t statusLedOffMs = 0;
uint32_t statusLedLastChangeMs = 0;

uint16_t clampPulse(uint16_t value, uint16_t low, uint16_t high);
int16_t clampInt16(int16_t value, int16_t low, int16_t high);
float clampFloat(float value, float low, float high);
float constrainFloat(float value, float minValue, float maxValue);
float absFloat(float value);
float mapPulseToFloat(uint16_t pulseUs, float low, float high);
PulseSnapshot copyChannel(volatile RxChannel &channel);
bool isPulseValid(const PulseSnapshot &pulse, uint32_t nowUs);
void handlePulseEdge(volatile RxChannel &channel, bool isHigh, uint32_t nowUs);
void steeringRxIsr();
void gainRxIsr();
void setupServoTimer50Hz();
void setServoPulseUs(uint16_t pulseUs);
bool writeMpuRegister(uint8_t reg, uint8_t value);
bool readMpuInt16(uint8_t reg, int16_t &value);
bool setupMpu6050();
void calibrateGyroZ();
float readGyroZDps(bool &ok);
void setupReceiverInputs();
void updateGainAndDamper(const PulseSnapshot &gain,
                         bool gainValid,
                         const PulseSnapshot &damper,
                         bool damperValid);
float calculateAdaptiveSmoothing(float baseSmoothing, float absGyroZ);
float calculateAdaptiveDeadband(float baseDeadbandDps, float absGyroZ);
float calculateProgressiveGainFactor(float absGyroZ);
float calculateDriverPriorityFactor(uint16_t receiverPulseUs);
uint16_t buildServoCommand(uint16_t receiverPulseUs, int16_t correctionUs);
void startRecalibration(uint32_t nowUs, uint16_t neutralUs);
void cancelRecalibration();
void finishRecalibration();
void updateRecalibration(uint32_t nowUs,
                         uint32_t nowMs,
                         const PulseSnapshot &recal,
                         bool recalValid,
                         const PulseSnapshot &steering,
                         bool steeringValid);
bool recalibrationOwnsServo();
void triggerStatusLedBlinks(uint8_t blinkCount, uint16_t onMs, uint16_t offMs);
void updateStatusLed(uint32_t nowMs);
void runControlLoop(uint32_t nowUs);
void debugPrint(bool radioOk,
                uint16_t steeringPulseUs,
                uint16_t gainPulseUs,
                bool gainValid,
                uint16_t damperPulseUs,
                bool damperValid,
                bool ch4Active,
                float gyroZDps,
                float absGyroZ,
                float effectiveSmoothing,
                float effectiveDeadband,
                float progressiveFactor,
                float effectiveGain,
                float correctionUs,
                float driverPriorityFactor,
                uint16_t servoPulseUs);

uint16_t clampPulse(uint16_t value, uint16_t low, uint16_t high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int16_t clampInt16(int16_t value, int16_t low, int16_t high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float constrainFloat(float value, float minValue, float maxValue) {
  return clampFloat(value, minValue, maxValue);
}

float absFloat(float value) {
  return value < 0.0f ? -value : value;
}

float mapPulseToFloat(uint16_t pulseUs, float low, float high) {
  uint16_t clipped = clampPulse(pulseUs, RX_COMMAND_MIN_US, RX_COMMAND_MAX_US);
  float ratio = (float)(clipped - RX_COMMAND_MIN_US) /
                (float)(RX_COMMAND_MAX_US - RX_COMMAND_MIN_US);
  return low + ratio * (high - low);
}

PulseSnapshot copyChannel(volatile RxChannel &channel) {
  PulseSnapshot copy;
  uint8_t oldSREG = SREG;
  noInterrupts();
  copy.pulseUs = channel.pulseUs;
  copy.lastPulseUs = channel.lastPulseUs;
  copy.hasPulse = channel.hasPulse;
  SREG = oldSREG;
  return copy;
}

bool isPulseValid(const PulseSnapshot &pulse, uint32_t nowUs) {
  return pulse.hasPulse && (uint32_t)(nowUs - pulse.lastPulseUs) <= RX_SIGNAL_TIMEOUT_US;
}

void handlePulseEdge(volatile RxChannel &channel, bool isHigh, uint32_t nowUs) {
  if (isHigh) {
    channel.riseUs = nowUs;
    return;
  }

  uint32_t width = nowUs - channel.riseUs;
  if (width >= RX_VALID_MIN_US && width <= RX_VALID_MAX_US) {
    channel.pulseUs = (uint16_t)width;
    channel.lastPulseUs = nowUs;
    channel.hasPulse = true;
  }
}

void steeringRxIsr() {
  handlePulseEdge(steeringChannel, digitalRead(RX_STEERING_PIN) == HIGH, micros());
}

void gainRxIsr() {
  handlePulseEdge(gainChannel, digitalRead(RX_GYRO_GAIN_PIN) == HIGH, micros());
}

ISR(PCINT2_vect) {
  uint8_t state = PIND;
  uint8_t changed = state ^ lastPortDState;
  uint32_t now = micros();

  if ((changed & RX_DAMPER_MASK) != 0) {
    handlePulseEdge(damperChannel, (state & RX_DAMPER_MASK) != 0, now);
  }

  if ((changed & RX_RECAL_MASK) != 0) {
    handlePulseEdge(recalChannel, (state & RX_RECAL_MASK) != 0, now);
  }

  lastPortDState = state;
}

ISR(TIMER1_COMPA_vect) {
  *servoPort |= servoBitMask;
  OCR1B = servoPulseTicks;
}

ISR(TIMER1_COMPB_vect) {
  *servoPort &= (uint8_t)~servoBitMask;
}

void setupServoTimer50Hz() {
  pinMode(SERVO_OUTPUT_PIN, OUTPUT);
  digitalWrite(SERVO_OUTPUT_PIN, LOW);

  servoPort = portOutputRegister(digitalPinToPort(SERVO_OUTPUT_PIN));
  servoBitMask = digitalPinToBitMask(SERVO_OUTPUT_PIN);

  uint8_t oldSREG = SREG;
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = SERVO_FRAME_TICKS - 1;
  OCR1B = SERVO_CENTER_US * TIMER1_TICKS_PER_US;
  TIMSK1 = 0;

  TCCR1B |= (1 << WGM12);  // CTC com TOP em OCR1A
  TCCR1B |= (1 << CS11);   // prescaler 8
  TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B);

  SREG = oldSREG;
}

void setServoPulseUs(uint16_t pulseUs) {
  uint16_t safePulse = clampPulse(pulseUs, SERVO_LEFT_LIMIT_US, SERVO_RIGHT_LIMIT_US);
  uint16_t ticks = safePulse * TIMER1_TICKS_PER_US;

  uint8_t oldSREG = SREG;
  noInterrupts();
  servoPulseTicks = ticks;
  SREG = oldSREG;
}

bool writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMpuInt16(uint8_t reg, int16_t &value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)2) != 2) {
    return false;
  }

  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();
  value = (int16_t)((highByte << 8) | lowByte);
  return true;
}

bool setupMpu6050() {
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  bool ok = true;
  ok &= writeMpuRegister(MPU_REG_PWR_MGMT_1, 0x01);   // acorda e usa clock do gyro X
  delay(100);
  ok &= writeMpuRegister(MPU_REG_CONFIG, MPU6050_DLPF_CFG);
  ok &= writeMpuRegister(MPU_REG_SMPLRT_DIV, 0x00);   // 1 kHz com DLPF ativo
  ok &= writeMpuRegister(MPU_REG_GYRO_CONFIG, 0x08);  // FS_SEL=1, +/-500 dps
  return ok;
}

void calibrateGyroZ() {
  int32_t sum = 0;
  uint16_t goodSamples = 0;
  int16_t rawZ = 0;

  if (DEBUG_SERIAL) {
    Serial.println(F("Calibrando gyro Z. Deixe o carro parado..."));
  }

  for (uint16_t i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    if (readMpuInt16(MPU_REG_GYRO_ZOUT_H, rawZ)) {
      sum += rawZ;
      goodSamples++;
    }
    delayMicroseconds(GYRO_CALIBRATION_INTERVAL_US);
  }

  if (goodSamples > 0) {
    gyroZOffsetRaw = (float)sum / (float)goodSamples;
  } else {
    gyroZOffsetRaw = 0.0f;
  }

  if (DEBUG_SERIAL) {
    Serial.print(F("Offset gyro Z raw = "));
    Serial.println(gyroZOffsetRaw);
  }
}

float readGyroZDps(bool &ok) {
  int16_t rawZ = 0;
  ok = readMpuInt16(MPU_REG_GYRO_ZOUT_H, rawZ);

  if (!ok) {
    return 0.0f;
  }

  return ((float)rawZ - gyroZOffsetRaw) / GYRO_LSB_PER_DPS;
}

void setupReceiverInputs() {
  pinMode(RX_STEERING_PIN, INPUT);
  pinMode(RX_GYRO_GAIN_PIN, INPUT);
  pinMode(RX_DAMPER_PIN, INPUT);
  pinMode(RX_RECAL_PIN, INPUT);

  int steeringInterrupt = digitalPinToInterrupt(RX_STEERING_PIN);
  if (steeringInterrupt != NOT_AN_INTERRUPT) {
    attachInterrupt(steeringInterrupt, steeringRxIsr, CHANGE);
  }

  int gainInterrupt = digitalPinToInterrupt(RX_GYRO_GAIN_PIN);
  if (gainInterrupt != NOT_AN_INTERRUPT) {
    attachInterrupt(gainInterrupt, gainRxIsr, CHANGE);
  }

  uint8_t oldSREG = SREG;
  noInterrupts();
  lastPortDState = PIND;
  PCICR |= _BV(PCIE2);
  PCMSK2 |= _BV(PCINT20) | _BV(PCINT23);
  SREG = oldSREG;
}

void updateGainAndDamper(const PulseSnapshot &gain,
                         bool gainValid,
                         const PulseSnapshot &damper,
                         bool damperValid) {
  if (gainValid) {
    if (gain.pulseUs < GYRO_GAIN_OFF_THRESHOLD_US) {
      activeGyroEnabled = false;
      activeGyroGain = 0.0f;
    } else {
      activeGyroEnabled = true;
      activeGyroGain = mapPulseToFloat(gain.pulseUs, GYRO_GAIN_MIN, GYRO_GAIN_MAX);
    }
  } else {
    activeGyroEnabled = true;
    activeGyroGain = GYRO_GAIN_FAILSAFE;
  }

  if (damperValid) {
    activeSmoothingAlpha =
      mapPulseToFloat(damper.pulseUs, DAMPER_SMOOTHING_MIN, DAMPER_SMOOTHING_MAX);
    activeGyroDeadbandDps =
      mapPulseToFloat(damper.pulseUs, DAMPER_DEADBAND_MIN_DPS, DAMPER_DEADBAND_MAX_DPS);
  } else {
    activeSmoothingAlpha = DAMPER_FAILSAFE_SMOOTHING;
    activeGyroDeadbandDps = DAMPER_FAILSAFE_DEADBAND_DPS;
  }
}

float calculateAdaptiveSmoothing(float baseSmoothing, float absGyroZ) {
  float lowRateSmoothing = baseSmoothing + ADAPTIVE_SMOOTHING_BOOST_AT_LOW_RATE;
  float highRateSmoothing = baseSmoothing - ADAPTIVE_SMOOTHING_REDUCTION_AT_HIGH_RATE;
  float effectiveSmoothing = lowRateSmoothing;

  if (absGyroZ >= ADAPTIVE_DAMPER_HIGH_DPS) {
    effectiveSmoothing = highRateSmoothing;
  } else if (absGyroZ > ADAPTIVE_DAMPER_LOW_DPS) {
    float t = (absGyroZ - ADAPTIVE_DAMPER_LOW_DPS) /
              (ADAPTIVE_DAMPER_HIGH_DPS - ADAPTIVE_DAMPER_LOW_DPS);
    effectiveSmoothing =
      lowRateSmoothing + t * (highRateSmoothing - lowRateSmoothing);
  }

  return constrainFloat(effectiveSmoothing, 0.55f, 0.85f);
}

float calculateAdaptiveDeadband(float baseDeadbandDps, float absGyroZ) {
  float lowRateDeadband =
    baseDeadbandDps + ADAPTIVE_DEADBAND_BOOST_AT_LOW_RATE_DPS;
  float highRateDeadband =
    baseDeadbandDps - ADAPTIVE_DEADBAND_REDUCTION_AT_HIGH_RATE_DPS;
  float effectiveDeadband = lowRateDeadband;

  if (absGyroZ >= ADAPTIVE_DAMPER_HIGH_DPS) {
    effectiveDeadband = highRateDeadband;
  } else if (absGyroZ > ADAPTIVE_DAMPER_LOW_DPS) {
    float t = (absGyroZ - ADAPTIVE_DAMPER_LOW_DPS) /
              (ADAPTIVE_DAMPER_HIGH_DPS - ADAPTIVE_DAMPER_LOW_DPS);
    effectiveDeadband =
      lowRateDeadband + t * (highRateDeadband - lowRateDeadband);
  }

  return constrainFloat(effectiveDeadband, 1.0f, 4.0f);
}

float calculateProgressiveGainFactor(float absGyroZ) {
  if (absGyroZ <= PROGRESSIVE_GAIN_START_DPS) {
    return PROGRESSIVE_GAIN_MIN_FACTOR;
  }

  if (absGyroZ >= PROGRESSIVE_GAIN_FULL_DPS) {
    return PROGRESSIVE_GAIN_MAX_FACTOR;
  }

  float t = (absGyroZ - PROGRESSIVE_GAIN_START_DPS) /
            (PROGRESSIVE_GAIN_FULL_DPS - PROGRESSIVE_GAIN_START_DPS);
  return PROGRESSIVE_GAIN_MIN_FACTOR +
         t * (PROGRESSIVE_GAIN_MAX_FACTOR - PROGRESSIVE_GAIN_MIN_FACTOR);
}

float calculateDriverPriorityFactor(uint16_t receiverPulseUs) {
  uint16_t steeringDistanceUs =
    abs((int16_t)receiverPulseUs - (int16_t)steeringNeutralUs);
  float ratio = clampFloat((float)steeringDistanceUs /
                             (float)DRIVER_PRIORITY_FULL_STEER_US,
                           0.0f,
                           1.0f);
  return 1.0f - ratio * (1.0f - DRIVER_PRIORITY_MIN_FACTOR);
}

uint16_t buildServoCommand(uint16_t receiverPulseUs, int16_t correctionUs) {
  int16_t steeringDeltaUs =
    (int16_t)receiverPulseUs - (int16_t)steeringNeutralUs;
  int16_t gyroCorrectionUs = correctionUs;

  if (SERVO_REVERSE) {
    steeringDeltaUs = -steeringDeltaUs;
    gyroCorrectionUs = -gyroCorrectionUs;
  }

  int16_t steeringUs = (int16_t)SERVO_CENTER_US + steeringDeltaUs;
  int16_t servoOutputUs = steeringUs + gyroCorrectionUs;
  servoOutputUs = clampInt16(servoOutputUs,
                             (int16_t)SERVO_LEFT_LIMIT_US,
                             (int16_t)SERVO_RIGHT_LIMIT_US);
  return (uint16_t)servoOutputUs;
}

void startRecalibration(uint32_t nowUs, uint16_t neutralUs) {
  recalState = RECAL_STABILITY_CHECK;
  recalNextSampleUs = nowUs;
  recalAttemptSamples = 0;
  recalGoodSamples = 0;
  recalRawSum = 0;
  recalPendingNeutralUs = neutralUs;
  filteredCorrectionUs = 0.0f;
  setServoPulseUs(SERVO_CENTER_US);

  if (DEBUG_SERIAL) {
    Serial.println(F("Recalibracao solicitada."));
  }
}

void cancelRecalibration() {
  recalState = RECAL_WAIT_RELEASE;
  recalAttemptSamples = 0;
  recalGoodSamples = 0;
  recalRawSum = 0;
  filteredCorrectionUs = 0.0f;
  setServoPulseUs(SERVO_CENTER_US);

  if (DEBUG_SERIAL) {
    Serial.println(F("Recalibracao cancelada."));
  }
}

void finishRecalibration() {
  bool recalibrationSucceeded = recalGoodSamples > 0;

  if (recalibrationSucceeded) {
    gyroZOffsetRaw = (float)recalRawSum / (float)recalGoodSamples;
    steeringNeutralUs = recalPendingNeutralUs;
  }

  recalState = RECAL_WAIT_RELEASE;
  recalAttemptSamples = 0;
  recalGoodSamples = 0;
  recalRawSum = 0;
  filteredCorrectionUs = 0.0f;
  setServoPulseUs(SERVO_CENTER_US);

  if (recalibrationSucceeded) {
    triggerStatusLedBlinks(3, STATUS_LED_FAST_ON_MS, STATUS_LED_FAST_OFF_MS);
  }

  if (DEBUG_SERIAL) {
    Serial.print(F("Recalibrado. Offset raw = "));
    Serial.print(gyroZOffsetRaw);
    Serial.print(F(" neutral = "));
    Serial.println(steeringNeutralUs);
  }
}

void updateRecalibration(uint32_t nowUs,
                         uint32_t nowMs,
                         const PulseSnapshot &recal,
                         bool recalValid,
                         const PulseSnapshot &steering,
                         bool steeringValid) {
  recalButtonActive = recalValid && recal.pulseUs >= RECAL_BUTTON_ACTIVE_US;

  if (recalState == RECAL_IDLE) {
    if (!recalButtonActive) {
      recalPressStartMs = 0;
      return;
    }

    if (recalPressStartMs == 0) {
      recalPressStartMs = nowMs;
      return;
    }

    if ((uint32_t)(nowMs - recalPressStartMs) < RECAL_HOLD_MS) {
      return;
    }

    if (!mpuReady || !steeringValid) {
      cancelRecalibration();
      return;
    }

    startRecalibration(nowUs,
                       clampPulse(steering.pulseUs,
                                  RX_COMMAND_MIN_US,
                                  RX_COMMAND_MAX_US));
    return;
  }

  if (recalState == RECAL_WAIT_RELEASE) {
    if (!recalButtonActive) {
      recalState = RECAL_IDLE;
      recalPressStartMs = 0;
    }
    return;
  }

  if ((uint32_t)(nowUs - recalNextSampleUs) < RECAL_SAMPLE_INTERVAL_US) {
    return;
  }

  recalNextSampleUs += RECAL_SAMPLE_INTERVAL_US;

  int16_t rawZ = 0;
  bool readOk = readMpuInt16(MPU_REG_GYRO_ZOUT_H, rawZ);
  recalAttemptSamples++;

  if (recalState == RECAL_STABILITY_CHECK) {
    if (readOk) {
      float gyroZDps = ((float)rawZ - gyroZOffsetRaw) / GYRO_LSB_PER_DPS;
      if (absFloat(gyroZDps) > RECAL_MAX_ABS_GYRO_DPS) {
        triggerStatusLedBlinks(2, STATUS_LED_LONG_ON_MS, STATUS_LED_LONG_OFF_MS);
        cancelRecalibration();
        return;
      }
      recalGoodSamples++;
    }

    if (recalAttemptSamples >= RECAL_STABILITY_SAMPLES) {
      if (recalGoodSamples < (RECAL_STABILITY_SAMPLES / 2)) {
        cancelRecalibration();
        return;
      }

      recalState = RECAL_SAMPLING_OFFSET;
      recalAttemptSamples = 0;
      recalGoodSamples = 0;
      recalRawSum = 0;
      recalNextSampleUs = nowUs;
    }
    return;
  }

  if (recalState == RECAL_SAMPLING_OFFSET) {
    if (readOk) {
      recalRawSum += rawZ;
      recalGoodSamples++;
    }

    if (recalAttemptSamples >= RECAL_OFFSET_SAMPLES) {
      if (recalGoodSamples == 0) {
        cancelRecalibration();
      } else {
        finishRecalibration();
      }
    }
  }
}

bool recalibrationOwnsServo() {
  return recalState == RECAL_STABILITY_CHECK || recalState == RECAL_SAMPLING_OFFSET;
}

void triggerStatusLedBlinks(uint8_t blinkCount, uint16_t onMs, uint16_t offMs) {
  if (blinkCount == 0) {
    return;
  }

  statusLedBlinkActive = true;
  statusLedIsOn = true;
  statusLedTargetBlinks = blinkCount;
  statusLedCompletedBlinks = 0;
  statusLedOnMs = onMs;
  statusLedOffMs = offMs;
  statusLedLastChangeMs = millis();
  digitalWrite(LED_BUILTIN, HIGH);
}

void updateStatusLed(uint32_t nowMs) {
  if (!statusLedBlinkActive) {
    if (statusLedIsOn) {
      statusLedIsOn = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
    return;
  }

  if (statusLedIsOn) {
    if ((uint32_t)(nowMs - statusLedLastChangeMs) < statusLedOnMs) {
      return;
    }

    statusLedIsOn = false;
    statusLedCompletedBlinks++;
    statusLedLastChangeMs = nowMs;
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  if ((uint32_t)(nowMs - statusLedLastChangeMs) < statusLedOffMs) {
    return;
  }

  if (statusLedCompletedBlinks >= statusLedTargetBlinks) {
    statusLedBlinkActive = false;
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  statusLedIsOn = true;
  statusLedLastChangeMs = nowMs;
  digitalWrite(LED_BUILTIN, HIGH);
}

void debugPrint(bool radioOk,
                uint16_t steeringPulseUs,
                uint16_t gainPulseUs,
                bool gainValid,
                uint16_t damperPulseUs,
                bool damperValid,
                bool ch4Active,
                float gyroZDps,
                float absGyroZ,
                float effectiveSmoothing,
                float effectiveDeadband,
                float progressiveFactor,
                float effectiveGain,
                float correctionUs,
                float driverPriorityFactor,
                uint16_t servoPulseUs) {
  if (!DEBUG_SERIAL) {
    return;
  }

  static uint32_t lastDebugMs = 0;
  uint32_t nowMs = millis();

  if ((uint32_t)(nowMs - lastDebugMs) < DEBUG_INTERVAL_MS) {
    return;
  }

  lastDebugMs = nowMs;

  Serial.print(F("radio="));
  Serial.print(radioOk ? F("OK") : F("FAIL"));
  Serial.print(F(" ch1="));
  Serial.print(steeringPulseUs);
  Serial.print(F(" ch5="));
  if (gainValid) {
    Serial.print(gainPulseUs);
  } else {
    Serial.print(F("FAIL"));
  }
  Serial.print(F(" gain="));
  Serial.print(activeGyroGain, 2);
  Serial.print(F(" ch6="));
  if (damperValid) {
    Serial.print(damperPulseUs);
  } else {
    Serial.print(F("FAIL"));
  }
  Serial.print(F(" smooth="));
  Serial.print(activeSmoothingAlpha, 2);
  Serial.print(F(" deadband="));
  Serial.print(activeGyroDeadbandDps, 2);
  Serial.print(F(" ch4="));
  Serial.print(ch4Active ? F("ON") : F("OFF"));
  Serial.print(F(" recal="));
  Serial.print((uint8_t)recalState);
  Serial.print(F(" gyroZ="));
  Serial.print(gyroZDps, 2);
  Serial.print(F(" absGyroZ="));
  Serial.print(absGyroZ, 2);
  Serial.print(F(" effSmooth="));
  Serial.print(effectiveSmoothing, 2);
  Serial.print(F(" effDeadband="));
  Serial.print(effectiveDeadband, 2);
  Serial.print(F(" prog="));
  Serial.print(progressiveFactor, 2);
  Serial.print(F(" effGain="));
  Serial.print(effectiveGain, 2);
  Serial.print(F(" corr="));
  Serial.print(correctionUs, 1);
  Serial.print(F(" priority="));
  Serial.print(driverPriorityFactor, 2);
  Serial.print(F(" out="));
  Serial.println(servoPulseUs);
}

void runControlLoop(uint32_t nowUs) {
  PulseSnapshot steering = copyChannel(steeringChannel);
  PulseSnapshot gain = copyChannel(gainChannel);
  PulseSnapshot damper = copyChannel(damperChannel);
  PulseSnapshot recal = copyChannel(recalChannel);

  bool steeringValid = isPulseValid(steering, nowUs);
  bool gainValid = isPulseValid(gain, nowUs);
  bool damperValid = isPulseValid(damper, nowUs);
  bool recalValid = isPulseValid(recal, nowUs);

  updateGainAndDamper(gain, gainValid, damper, damperValid);
  updateRecalibration(nowUs, millis(), recal, recalValid, steering, steeringValid);

  if (!steeringValid) {
    filteredCorrectionUs = 0.0f;
    setServoPulseUs(SERVO_CENTER_US);
    debugPrint(false,
               SERVO_CENTER_US,
               gain.pulseUs,
               gainValid,
               damper.pulseUs,
               damperValid,
               recalButtonActive,
               0.0f,
               0.0f,
               activeSmoothingAlpha,
               activeGyroDeadbandDps,
               PROGRESSIVE_GAIN_MIN_FACTOR,
               0.0f,
               0.0f,
               1.0f,
               SERVO_CENTER_US);
    return;
  }

  uint16_t receiverPulseUs =
    clampPulse(steering.pulseUs, RX_COMMAND_MIN_US, RX_COMMAND_MAX_US);

  if (recalibrationOwnsServo()) {
    filteredCorrectionUs = 0.0f;
    setServoPulseUs(SERVO_CENTER_US);
    debugPrint(true,
               receiverPulseUs,
               gain.pulseUs,
               gainValid,
               damper.pulseUs,
               damperValid,
               recalButtonActive,
               0.0f,
               0.0f,
               activeSmoothingAlpha,
               activeGyroDeadbandDps,
               PROGRESSIVE_GAIN_MIN_FACTOR,
               0.0f,
               0.0f,
               1.0f,
               SERVO_CENTER_US);
    return;
  }

  bool gyroOk = false;
  float gyroZDps = mpuReady ? readGyroZDps(gyroOk) : 0.0f;

  if (!gyroOk) {
    gyroZDps = 0.0f;
  }

  float absGyroZ = absFloat(gyroZDps);
  float effectiveSmoothing =
    calculateAdaptiveSmoothing(activeSmoothingAlpha, absGyroZ);
  float effectiveDeadband =
    calculateAdaptiveDeadband(activeGyroDeadbandDps, absGyroZ);
  float progressiveFactor = calculateProgressiveGainFactor(absGyroZ);
  float effectiveGain = activeGyroGain * progressiveFactor;
  float driverPriorityFactor = calculateDriverPriorityFactor(receiverPulseUs);
  float activeCorrectionLimitUs =
    (float)GYRO_CORRECTION_LIMIT_US * driverPriorityFactor;

  if (!activeGyroEnabled) {
    filteredCorrectionUs = 0.0f;
    effectiveGain = 0.0f;
  } else {
    float gyroForCorrectionDps = gyroZDps;
    if (gyroForCorrectionDps > -effectiveDeadband &&
        gyroForCorrectionDps < effectiveDeadband) {
      gyroForCorrectionDps = 0.0f;
    }

    float correctionUs = gyroForCorrectionDps * effectiveGain;
    if (GYRO_REVERSE) {
      correctionUs = -correctionUs;
    }

    correctionUs = clampFloat(correctionUs,
                              -(float)GYRO_CORRECTION_LIMIT_US,
                              (float)GYRO_CORRECTION_LIMIT_US);
    correctionUs = clampFloat(correctionUs,
                              -activeCorrectionLimitUs,
                              activeCorrectionLimitUs);

    filteredCorrectionUs =
      filteredCorrectionUs * effectiveSmoothing +
      correctionUs * (1.0f - effectiveSmoothing);

    filteredCorrectionUs = clampFloat(filteredCorrectionUs,
                                      -activeCorrectionLimitUs,
                                      activeCorrectionLimitUs);
  }

  int16_t gyroCorrectionUs = (int16_t)filteredCorrectionUs;
  gyroCorrectionUs = clampInt16(gyroCorrectionUs,
                                -GYRO_CORRECTION_LIMIT_US,
                                GYRO_CORRECTION_LIMIT_US);

  uint16_t servoPulseUs = buildServoCommand(receiverPulseUs, gyroCorrectionUs);
  setServoPulseUs(servoPulseUs);

  debugPrint(true,
             receiverPulseUs,
             gain.pulseUs,
             gainValid,
             damper.pulseUs,
             damperValid,
             recalButtonActive,
             gyroZDps,
             absGyroZ,
             effectiveSmoothing,
             effectiveDeadband,
             progressiveFactor,
             effectiveGain,
             filteredCorrectionUs,
             driverPriorityFactor,
             servoPulseUs);
}

void setup() {
  if (DEBUG_SERIAL) {
    Serial.begin(DEBUG_BAUD);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  setupServoTimer50Hz();
  setServoPulseUs(SERVO_CENTER_US);
  setupReceiverInputs();

  mpuReady = setupMpu6050();
  if (mpuReady) {
    calibrateGyroZ();
  }

  if (DEBUG_SERIAL) {
    Serial.print(F("MPU6050: "));
    Serial.println(mpuReady ? F("OK") : F("FALHA"));
    Serial.println(F("Firmware pronto."));
  }
}

void loop() {
  updateStatusLed(millis());

  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_INTERVAL_US) {
    lastControlUs = nowUs;
    runControlLoop(nowUs);
  }
}
