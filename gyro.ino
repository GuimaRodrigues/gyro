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
const uint8_t RX_STEERING_FILTER_SHIFT = 1;  // 1 = 50% pulso novo, reduz jitter sem muito atraso
const uint8_t RX_AUX_FILTER_SHIFT = 2;       // 2 = 25% pulso novo, bom para knobs/botao

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
const uint16_t GYRO_CALIBRATION_SAMPLES = 1500;
const uint16_t GYRO_CALIBRATION_INTERVAL_US = 1000;
const uint16_t CONTROL_INTERVAL_US = 1000;  // controle a 1 kHz
const uint32_t I2C_TIMEOUT_US = 3000UL;
const uint16_t MPU6050_WAKE_DELAY_MS = 100;
const uint8_t CALIBRATION_MIN_GOOD_PERCENT = 80;

// Escalas inteiras usadas no loop de controle para aliviar o ATmega328P.
const int16_t FIXED_SCALE = 1000;
const int16_t DPS_SCALE = 100;              // graus/s em centesimos
const int16_t GYRO_RAW_TO_CENTI_DPS_NUM = 1000;
const int16_t GYRO_RAW_TO_CENTI_DPS_DEN = 655;

// Inverta se o gyro corrigir para o lado errado.
const bool GYRO_REVERSE = false;

const int16_t GYRO_GAIN_MIN_Q1000 = 0;
const int16_t GYRO_GAIN_MAX_Q1000 = 3000;
const uint16_t GYRO_GAIN_OFF_THRESHOLD_US = 1080;
const int16_t GYRO_GAIN_FAILSAFE_Q1000 = 1500;

const int16_t DAMPER_SMOOTHING_MIN_Q1000 = 600;
const int16_t DAMPER_SMOOTHING_MAX_Q1000 = 800;
const int16_t DAMPER_DEADBAND_MIN_CENTI_DPS = 150;
const int16_t DAMPER_DEADBAND_MAX_CENTI_DPS = 300;
const int16_t DAMPER_FAILSAFE_SMOOTHING_Q1000 = 700;
const int16_t DAMPER_FAILSAFE_DEADBAND_CENTI_DPS = 200;

const int16_t ADAPTIVE_DAMPER_LOW_CENTI_DPS = 800;
const int16_t ADAPTIVE_DAMPER_HIGH_CENTI_DPS = 8000;
const int16_t ADAPTIVE_SMOOTHING_BOOST_AT_LOW_RATE_Q1000 = 60;
const int16_t ADAPTIVE_SMOOTHING_REDUCTION_AT_HIGH_RATE_Q1000 = 50;
const int16_t ADAPTIVE_DEADBAND_BOOST_AT_LOW_RATE_CENTI_DPS = 60;
const int16_t ADAPTIVE_DEADBAND_REDUCTION_AT_HIGH_RATE_CENTI_DPS = 40;

const int16_t PROGRESSIVE_GAIN_START_CENTI_DPS = 2000;
const int16_t PROGRESSIVE_GAIN_FULL_CENTI_DPS = 12000;
const int16_t PROGRESSIVE_GAIN_MIN_FACTOR_Q1000 = 750;
const int16_t PROGRESSIVE_GAIN_MAX_FACTOR_Q1000 = 1180;

const int16_t GYRO_CORRECTION_LIMIT_US = 250;
const uint16_t DRIVER_PRIORITY_FULL_STEER_US = 230;
const int16_t DRIVER_PRIORITY_MIN_FACTOR_Q1000 = 550;

// ------------------------- Recalibracao CH4 -------------------------
const uint16_t RECAL_BUTTON_ACTIVE_US = 1700;
const uint16_t RECAL_HOLD_MS = 2500;
const int16_t RECAL_MAX_ABS_GYRO_CENTI_DPS = 500;
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

const uint8_t RX_STEERING_MASK = _BV(PD2);
const uint8_t RX_GAIN_MASK = _BV(PD3);
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
  volatile uint16_t filteredPulseQ4;
  volatile uint32_t lastPulseUs;
  volatile bool hasPulse;
  uint8_t filterShift;
};

struct ReceiverSnapshot {
  PulseSnapshot steering;
  PulseSnapshot gain;
  PulseSnapshot damper;
  PulseSnapshot recal;
  bool steeringValid;
  bool gainValid;
  bool damperValid;
  bool recalValid;
};

struct ControlTerms {
  int32_t gyroZCentiDps;
  int32_t absGyroCentiDps;
  int16_t effectiveSmoothingQ1000;
  int16_t effectiveDeadbandCentiDps;
  int16_t progressiveFactorQ1000;
  int16_t effectiveGainQ1000;
  int16_t driverPriorityFactorQ1000;
  int32_t activeCorrectionLimitUs;
};

enum RecalState {
  RECAL_IDLE,
  RECAL_STABILITY_CHECK,
  RECAL_SAMPLING_OFFSET,
  RECAL_WAIT_RELEASE
};

volatile RxChannel steeringChannel = {
  0, RX_COMMAND_CENTER_US, RX_COMMAND_CENTER_US * 16, 0, false, RX_STEERING_FILTER_SHIFT
};
volatile RxChannel gainChannel = {
  0, RX_COMMAND_CENTER_US, RX_COMMAND_CENTER_US * 16, 0, false, RX_AUX_FILTER_SHIFT
};
volatile RxChannel damperChannel = {
  0, RX_COMMAND_CENTER_US, RX_COMMAND_CENTER_US * 16, 0, false, RX_AUX_FILTER_SHIFT
};
volatile RxChannel recalChannel = {
  0, RX_COMMAND_CENTER_US, RX_COMMAND_CENTER_US * 16, 0, false, RX_AUX_FILTER_SHIFT
};
volatile uint8_t lastPortDState = 0;

volatile uint16_t servoPulseTicks = SERVO_CENTER_US * TIMER1_TICKS_PER_US;
volatile uint8_t *servoPort = 0;
uint8_t servoBitMask = 0;

bool mpuReady = false;
bool mpuSetupPending = false;
bool initialGyroCalibrated = false;
bool initialGyroCalibrationActive = false;
int16_t gyroZOffsetRaw = 0;
int32_t filteredCorrectionUsQ1000 = 0;
int16_t activeGyroGainQ1000 = GYRO_GAIN_FAILSAFE_Q1000;
int16_t activeSmoothingQ1000 = DAMPER_FAILSAFE_SMOOTHING_Q1000;
int16_t activeGyroDeadbandCentiDps = DAMPER_FAILSAFE_DEADBAND_CENTI_DPS;
bool activeGyroEnabled = true;
uint16_t steeringNeutralUs = RX_COMMAND_CENTER_US;
uint32_t lastControlUs = 0;
uint32_t mpuWakeStartMs = 0;
uint32_t initialCalNextSampleUs = 0;
uint16_t initialCalAttemptSamples = 0;
uint16_t initialCalGoodSamples = 0;
int32_t initialCalRawSum = 0;

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
int32_t clampInt32(int32_t value, int32_t low, int32_t high);
int16_t mapPulseToScaled(uint16_t pulseUs, int16_t low, int16_t high);
int32_t absInt32(int32_t value);
bool hasEnoughGoodSamples(uint16_t goodSamples, uint16_t totalSamples);
int32_t rawGyroToCentiDps(int16_t rawZ);
PulseSnapshot copyChannel(volatile RxChannel &channel);
bool isPulseValid(const PulseSnapshot &pulse, uint32_t nowUs);
ReceiverSnapshot readReceiverSnapshot(uint32_t nowUs);
void handlePulseEdge(volatile RxChannel &channel, bool isHigh, uint32_t nowUs);
void steeringRxIsr();
void gainRxIsr();
void setupServoTimer50Hz();
void setServoPulseUs(uint16_t pulseUs);
bool writeMpuRegister(uint8_t reg, uint8_t value);
bool readMpuInt16(uint8_t reg, int16_t &value);
bool startMpu6050();
void updateMpu6050Setup(uint32_t nowMs, uint32_t nowUs);
void startInitialGyroCalibration(uint32_t nowUs);
void updateInitialGyroCalibration(uint32_t nowUs);
int32_t readGyroZCentiDps(bool &ok);
void setupReceiverInputs();
void updateGainAndDamper(const PulseSnapshot &gain,
                         bool gainValid,
                         const PulseSnapshot &damper,
                         bool damperValid);
int16_t calculateAdaptiveSmoothingQ1000(int16_t baseSmoothingQ1000,
                                        int32_t absGyroCentiDps);
int16_t calculateAdaptiveDeadbandCentiDps(int16_t baseDeadbandCentiDps,
                                          int32_t absGyroCentiDps);
int16_t calculateProgressiveGainFactorQ1000(int32_t absGyroCentiDps);
int16_t calculateDriverPriorityFactorQ1000(uint16_t receiverPulseUs);
ControlTerms buildControlTerms(uint16_t receiverPulseUs, int32_t gyroZCentiDps);
int16_t updateGyroCorrectionUs(ControlTerms &terms);
uint16_t buildServoCommand(uint16_t receiverPulseUs, int16_t correctionUs);
void setCenteredSafeOutput();
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
void runControlLoop(uint32_t nowUs, uint32_t nowMs);
void debugSafeControl(bool radioOk,
                      const ReceiverSnapshot &rx,
                      uint16_t steeringPulseUs);
void printScaled(int32_t value, uint16_t scale, uint8_t decimals);
void debugPrint(bool radioOk,
                uint16_t steeringPulseUs,
                uint16_t gainPulseUs,
                bool gainValid,
                uint16_t damperPulseUs,
                bool damperValid,
                bool ch4Active,
                int32_t gyroZCentiDps,
                int32_t absGyroCentiDps,
                int16_t effectiveSmoothingQ1000,
                int16_t effectiveDeadbandCentiDps,
                int16_t progressiveFactorQ1000,
                int16_t effectiveGainQ1000,
                int32_t correctionUsQ1000,
                int16_t driverPriorityFactorQ1000,
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

int32_t clampInt32(int32_t value, int32_t low, int32_t high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int16_t mapPulseToScaled(uint16_t pulseUs, int16_t low, int16_t high) {
  uint16_t clipped = clampPulse(pulseUs, RX_COMMAND_MIN_US, RX_COMMAND_MAX_US);
  int32_t span = (int32_t)high - (int32_t)low;
  int32_t offset = (int32_t)clipped - (int32_t)RX_COMMAND_MIN_US;
  return (int16_t)(low + (span * offset) /
                         (RX_COMMAND_MAX_US - RX_COMMAND_MIN_US));
}

int32_t absInt32(int32_t value) {
  return value < 0 ? -value : value;
}

bool hasEnoughGoodSamples(uint16_t goodSamples, uint16_t totalSamples) {
  return ((uint32_t)goodSamples * 100UL) >=
         ((uint32_t)totalSamples * CALIBRATION_MIN_GOOD_PERCENT);
}

int32_t rawGyroToCentiDps(int16_t rawZ) {
  int32_t rawDelta = (int32_t)rawZ - (int32_t)gyroZOffsetRaw;
  return (rawDelta * GYRO_RAW_TO_CENTI_DPS_NUM) / GYRO_RAW_TO_CENTI_DPS_DEN;
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

ReceiverSnapshot readReceiverSnapshot(uint32_t nowUs) {
  ReceiverSnapshot rx;
  rx.steering = copyChannel(steeringChannel);
  rx.gain = copyChannel(gainChannel);
  rx.damper = copyChannel(damperChannel);
  rx.recal = copyChannel(recalChannel);
  rx.steeringValid = isPulseValid(rx.steering, nowUs);
  rx.gainValid = isPulseValid(rx.gain, nowUs);
  rx.damperValid = isPulseValid(rx.damper, nowUs);
  rx.recalValid = isPulseValid(rx.recal, nowUs);
  return rx;
}

void handlePulseEdge(volatile RxChannel &channel, bool isHigh, uint32_t nowUs) {
  if (isHigh) {
    channel.riseUs = nowUs;
    return;
  }

  uint32_t width = nowUs - channel.riseUs;
  if (width >= RX_VALID_MIN_US && width <= RX_VALID_MAX_US) {
    uint16_t filteredPulseUs = (uint16_t)width;

    if (!channel.hasPulse || channel.filterShift == 0) {
      channel.filteredPulseQ4 = filteredPulseUs * 16;
    } else {
      int32_t targetQ4 = (int32_t)filteredPulseUs * 16L;
      int32_t filteredQ4 = channel.filteredPulseQ4;
      int32_t deltaQ4 = targetQ4 - filteredQ4;
      int32_t adjustmentQ4 = absInt32(deltaQ4) >> channel.filterShift;

      if (deltaQ4 < 0) {
        filteredQ4 -= adjustmentQ4;
      } else {
        filteredQ4 += adjustmentQ4;
      }

      filteredQ4 = clampInt32(filteredQ4,
                              (int32_t)RX_VALID_MIN_US * 16L,
                              (int32_t)RX_VALID_MAX_US * 16L);
      channel.filteredPulseQ4 = (uint16_t)filteredQ4;
      filteredPulseUs = (uint16_t)((filteredQ4 + 8) / 16);
    }

    channel.pulseUs = filteredPulseUs;
    channel.lastPulseUs = nowUs;
    channel.hasPulse = true;
  }
}

void steeringRxIsr() {
  uint8_t state = PIND;
  handlePulseEdge(steeringChannel, (state & RX_STEERING_MASK) != 0, micros());
}

void gainRxIsr() {
  uint8_t state = PIND;
  handlePulseEdge(gainChannel, (state & RX_GAIN_MASK) != 0, micros());
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

bool startMpu6050() {
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
#endif

  if (!writeMpuRegister(MPU_REG_PWR_MGMT_1, 0x01)) {  // acorda e usa clock do gyro X
    return false;
  }

  mpuSetupPending = true;
  mpuWakeStartMs = millis();
  mpuReady = false;
  initialGyroCalibrated = false;
  return true;
}

void updateMpu6050Setup(uint32_t nowMs, uint32_t nowUs) {
  if (!mpuSetupPending) {
    return;
  }

  if ((uint32_t)(nowMs - mpuWakeStartMs) < MPU6050_WAKE_DELAY_MS) {
    return;
  }

  bool ok = true;
  ok &= writeMpuRegister(MPU_REG_CONFIG, MPU6050_DLPF_CFG);
  ok &= writeMpuRegister(MPU_REG_SMPLRT_DIV, 0x00);   // 1 kHz com DLPF ativo
  ok &= writeMpuRegister(MPU_REG_GYRO_CONFIG, 0x08);  // FS_SEL=1, +/-500 dps

  mpuSetupPending = false;
  mpuReady = ok;

  if (mpuReady) {
    startInitialGyroCalibration(nowUs);
  } else {
    initialGyroCalibrated = true;
  }

  if (DEBUG_SERIAL) {
    Serial.print(F("MPU6050: "));
    Serial.println(mpuReady ? F("OK") : F("FALHA"));
  }
}

void startInitialGyroCalibration(uint32_t nowUs) {
  initialGyroCalibrationActive = true;
  initialGyroCalibrated = false;
  initialCalNextSampleUs = nowUs;
  initialCalAttemptSamples = 0;
  initialCalGoodSamples = 0;
  initialCalRawSum = 0;
  filteredCorrectionUsQ1000 = 0;
  setServoPulseUs(SERVO_CENTER_US);

  if (DEBUG_SERIAL) {
    Serial.println(F("Calibrando gyro Z. Deixe o carro parado..."));
  }
}

void updateInitialGyroCalibration(uint32_t nowUs) {
  if (!initialGyroCalibrationActive) {
    return;
  }

  if ((uint32_t)(nowUs - initialCalNextSampleUs) < GYRO_CALIBRATION_INTERVAL_US) {
    return;
  }

  initialCalNextSampleUs += GYRO_CALIBRATION_INTERVAL_US;

  int16_t rawZ = 0;
  if (readMpuInt16(MPU_REG_GYRO_ZOUT_H, rawZ)) {
    initialCalRawSum += rawZ;
    initialCalGoodSamples++;
  }

  initialCalAttemptSamples++;

  if (initialCalAttemptSamples < GYRO_CALIBRATION_SAMPLES) {
    return;
  }

  if (hasEnoughGoodSamples(initialCalGoodSamples, GYRO_CALIBRATION_SAMPLES)) {
    gyroZOffsetRaw = (int16_t)(initialCalRawSum / (int32_t)initialCalGoodSamples);
  } else {
    gyroZOffsetRaw = 0;
    mpuReady = false;
  }

  initialGyroCalibrationActive = false;
  initialGyroCalibrated = true;

  if (DEBUG_SERIAL) {
    Serial.print(F("Offset gyro Z raw = "));
    Serial.println(gyroZOffsetRaw);
  }
}

int32_t readGyroZCentiDps(bool &ok) {
  int16_t rawZ = 0;
  ok = readMpuInt16(MPU_REG_GYRO_ZOUT_H, rawZ);

  if (!ok) {
    return 0;
  }

  return rawGyroToCentiDps(rawZ);
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
      activeGyroGainQ1000 = 0;
    } else {
      activeGyroEnabled = true;
      activeGyroGainQ1000 =
        mapPulseToScaled(gain.pulseUs, GYRO_GAIN_MIN_Q1000, GYRO_GAIN_MAX_Q1000);
    }
  } else {
    activeGyroEnabled = true;
    activeGyroGainQ1000 = GYRO_GAIN_FAILSAFE_Q1000;
  }

  if (damperValid) {
    activeSmoothingQ1000 =
      mapPulseToScaled(damper.pulseUs,
                       DAMPER_SMOOTHING_MIN_Q1000,
                       DAMPER_SMOOTHING_MAX_Q1000);
    activeGyroDeadbandCentiDps =
      mapPulseToScaled(damper.pulseUs,
                       DAMPER_DEADBAND_MIN_CENTI_DPS,
                       DAMPER_DEADBAND_MAX_CENTI_DPS);
  } else {
    activeSmoothingQ1000 = DAMPER_FAILSAFE_SMOOTHING_Q1000;
    activeGyroDeadbandCentiDps = DAMPER_FAILSAFE_DEADBAND_CENTI_DPS;
  }
}

int16_t calculateAdaptiveSmoothingQ1000(int16_t baseSmoothingQ1000,
                                        int32_t absGyroCentiDps) {
  int16_t lowRateSmoothing =
    baseSmoothingQ1000 + ADAPTIVE_SMOOTHING_BOOST_AT_LOW_RATE_Q1000;
  int16_t highRateSmoothing =
    baseSmoothingQ1000 - ADAPTIVE_SMOOTHING_REDUCTION_AT_HIGH_RATE_Q1000;
  int32_t effectiveSmoothing = lowRateSmoothing;

  if (absGyroCentiDps >= ADAPTIVE_DAMPER_HIGH_CENTI_DPS) {
    effectiveSmoothing = highRateSmoothing;
  } else if (absGyroCentiDps > ADAPTIVE_DAMPER_LOW_CENTI_DPS) {
    int32_t tNum = absGyroCentiDps - ADAPTIVE_DAMPER_LOW_CENTI_DPS;
    int32_t tDen = ADAPTIVE_DAMPER_HIGH_CENTI_DPS - ADAPTIVE_DAMPER_LOW_CENTI_DPS;
    effectiveSmoothing =
      lowRateSmoothing +
      ((int32_t)(highRateSmoothing - lowRateSmoothing) * tNum) / tDen;
  }

  return (int16_t)clampInt32(effectiveSmoothing, 550, 850);
}

int16_t calculateAdaptiveDeadbandCentiDps(int16_t baseDeadbandCentiDps,
                                          int32_t absGyroCentiDps) {
  int16_t lowRateDeadband =
    baseDeadbandCentiDps + ADAPTIVE_DEADBAND_BOOST_AT_LOW_RATE_CENTI_DPS;
  int16_t highRateDeadband =
    baseDeadbandCentiDps - ADAPTIVE_DEADBAND_REDUCTION_AT_HIGH_RATE_CENTI_DPS;
  int32_t effectiveDeadband = lowRateDeadband;

  if (absGyroCentiDps >= ADAPTIVE_DAMPER_HIGH_CENTI_DPS) {
    effectiveDeadband = highRateDeadband;
  } else if (absGyroCentiDps > ADAPTIVE_DAMPER_LOW_CENTI_DPS) {
    int32_t tNum = absGyroCentiDps - ADAPTIVE_DAMPER_LOW_CENTI_DPS;
    int32_t tDen = ADAPTIVE_DAMPER_HIGH_CENTI_DPS - ADAPTIVE_DAMPER_LOW_CENTI_DPS;
    effectiveDeadband =
      lowRateDeadband +
      ((int32_t)(highRateDeadband - lowRateDeadband) * tNum) / tDen;
  }

  return (int16_t)clampInt32(effectiveDeadband, 100, 400);
}

int16_t calculateProgressiveGainFactorQ1000(int32_t absGyroCentiDps) {
  if (absGyroCentiDps <= PROGRESSIVE_GAIN_START_CENTI_DPS) {
    return PROGRESSIVE_GAIN_MIN_FACTOR_Q1000;
  }

  if (absGyroCentiDps >= PROGRESSIVE_GAIN_FULL_CENTI_DPS) {
    return PROGRESSIVE_GAIN_MAX_FACTOR_Q1000;
  }

  int32_t tNum = absGyroCentiDps - PROGRESSIVE_GAIN_START_CENTI_DPS;
  int32_t tDen = PROGRESSIVE_GAIN_FULL_CENTI_DPS - PROGRESSIVE_GAIN_START_CENTI_DPS;
  return (int16_t)(PROGRESSIVE_GAIN_MIN_FACTOR_Q1000 +
                   ((int32_t)(PROGRESSIVE_GAIN_MAX_FACTOR_Q1000 -
                              PROGRESSIVE_GAIN_MIN_FACTOR_Q1000) *
                    tNum) / tDen);
}

int16_t calculateDriverPriorityFactorQ1000(uint16_t receiverPulseUs) {
  uint16_t steeringDistanceUs =
    abs((int16_t)receiverPulseUs - (int16_t)steeringNeutralUs);
  steeringDistanceUs = clampPulse(steeringDistanceUs,
                                  0,
                                  DRIVER_PRIORITY_FULL_STEER_US);
  int32_t reductionRange = FIXED_SCALE - DRIVER_PRIORITY_MIN_FACTOR_Q1000;
  int32_t reduction =
    (reductionRange * steeringDistanceUs) / DRIVER_PRIORITY_FULL_STEER_US;
  return (int16_t)(FIXED_SCALE - reduction);
}

ControlTerms buildControlTerms(uint16_t receiverPulseUs, int32_t gyroZCentiDps) {
  ControlTerms terms;
  terms.gyroZCentiDps = gyroZCentiDps;
  terms.absGyroCentiDps = absInt32(gyroZCentiDps);
  terms.effectiveSmoothingQ1000 =
    calculateAdaptiveSmoothingQ1000(activeSmoothingQ1000, terms.absGyroCentiDps);
  terms.effectiveDeadbandCentiDps =
    calculateAdaptiveDeadbandCentiDps(activeGyroDeadbandCentiDps,
                                      terms.absGyroCentiDps);
  terms.progressiveFactorQ1000 =
    calculateProgressiveGainFactorQ1000(terms.absGyroCentiDps);
  terms.effectiveGainQ1000 =
    (int16_t)(((int32_t)activeGyroGainQ1000 * terms.progressiveFactorQ1000) /
              FIXED_SCALE);
  terms.driverPriorityFactorQ1000 =
    calculateDriverPriorityFactorQ1000(receiverPulseUs);
  terms.activeCorrectionLimitUs =
    ((int32_t)GYRO_CORRECTION_LIMIT_US * terms.driverPriorityFactorQ1000) /
    FIXED_SCALE;
  return terms;
}

int16_t updateGyroCorrectionUs(ControlTerms &terms) {
  if (!activeGyroEnabled) {
    filteredCorrectionUsQ1000 = 0;
    terms.effectiveGainQ1000 = 0;
  } else {
    int32_t gyroForCorrectionCentiDps = terms.gyroZCentiDps;
    if (gyroForCorrectionCentiDps > -terms.effectiveDeadbandCentiDps &&
        gyroForCorrectionCentiDps < terms.effectiveDeadbandCentiDps) {
      gyroForCorrectionCentiDps = 0;
    }

    int32_t correctionUs =
      (gyroForCorrectionCentiDps * terms.effectiveGainQ1000) /
      ((int32_t)DPS_SCALE * FIXED_SCALE);
    if (GYRO_REVERSE) {
      correctionUs = -correctionUs;
    }

    correctionUs = clampInt32(correctionUs,
                              -GYRO_CORRECTION_LIMIT_US,
                              GYRO_CORRECTION_LIMIT_US);
    correctionUs = clampInt32(correctionUs,
                              -terms.activeCorrectionLimitUs,
                              terms.activeCorrectionLimitUs);

    int32_t correctionUsQ1000 = correctionUs * (int32_t)FIXED_SCALE;
    filteredCorrectionUsQ1000 =
      (filteredCorrectionUsQ1000 * terms.effectiveSmoothingQ1000 +
       correctionUsQ1000 * (FIXED_SCALE - terms.effectiveSmoothingQ1000)) /
      FIXED_SCALE;

    filteredCorrectionUsQ1000 =
      clampInt32(filteredCorrectionUsQ1000,
                 -terms.activeCorrectionLimitUs * (int32_t)FIXED_SCALE,
                 terms.activeCorrectionLimitUs * (int32_t)FIXED_SCALE);
  }

  int32_t roundedCorrectionUs =
    filteredCorrectionUsQ1000 >= 0 ?
      (filteredCorrectionUsQ1000 + (FIXED_SCALE / 2)) / FIXED_SCALE :
      (filteredCorrectionUsQ1000 - (FIXED_SCALE / 2)) / FIXED_SCALE;
  int16_t gyroCorrectionUs = (int16_t)roundedCorrectionUs;
  return clampInt16(gyroCorrectionUs,
                    -GYRO_CORRECTION_LIMIT_US,
                    GYRO_CORRECTION_LIMIT_US);
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

void setCenteredSafeOutput() {
  filteredCorrectionUsQ1000 = 0;
  setServoPulseUs(SERVO_CENTER_US);
}

void startRecalibration(uint32_t nowUs, uint16_t neutralUs) {
  recalState = RECAL_STABILITY_CHECK;
  recalNextSampleUs = nowUs;
  recalAttemptSamples = 0;
  recalGoodSamples = 0;
  recalRawSum = 0;
  recalPendingNeutralUs = neutralUs;
  filteredCorrectionUsQ1000 = 0;
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
  filteredCorrectionUsQ1000 = 0;
  setServoPulseUs(SERVO_CENTER_US);

  if (DEBUG_SERIAL) {
    Serial.println(F("Recalibracao cancelada."));
  }
}

void finishRecalibration() {
  bool recalibrationSucceeded =
    hasEnoughGoodSamples(recalGoodSamples, RECAL_OFFSET_SAMPLES);

  if (recalibrationSucceeded) {
    gyroZOffsetRaw = (int16_t)(recalRawSum / (int32_t)recalGoodSamples);
    steeringNeutralUs = recalPendingNeutralUs;
  }

  recalState = RECAL_WAIT_RELEASE;
  recalAttemptSamples = 0;
  recalGoodSamples = 0;
  recalRawSum = 0;
  filteredCorrectionUsQ1000 = 0;
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
      int32_t gyroZCentiDps = rawGyroToCentiDps(rawZ);
      if (absInt32(gyroZCentiDps) > RECAL_MAX_ABS_GYRO_CENTI_DPS) {
        triggerStatusLedBlinks(2, STATUS_LED_LONG_ON_MS, STATUS_LED_LONG_OFF_MS);
        cancelRecalibration();
        return;
      }
      recalGoodSamples++;
    }

    if (recalAttemptSamples >= RECAL_STABILITY_SAMPLES) {
      if (!hasEnoughGoodSamples(recalGoodSamples, RECAL_STABILITY_SAMPLES)) {
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
      if (!hasEnoughGoodSamples(recalGoodSamples, RECAL_OFFSET_SAMPLES)) {
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

void printScaled(int32_t value, uint16_t scale, uint8_t decimals) {
  if (value < 0) {
    Serial.print('-');
    value = -value;
  }

  Serial.print(value / scale);

  if (decimals == 0) {
    return;
  }

  Serial.print('.');
  int32_t remainder = value % scale;
  uint16_t divisor = scale / 10;

  for (uint8_t i = 0; i < decimals; i++) {
    uint8_t digit = 0;
    if (divisor > 0) {
      digit = (uint8_t)(remainder / divisor);
      remainder %= divisor;
      divisor /= 10;
    }
    Serial.print(digit);
  }
}

void debugPrint(bool radioOk,
                uint16_t steeringPulseUs,
                uint16_t gainPulseUs,
                bool gainValid,
                uint16_t damperPulseUs,
                bool damperValid,
                bool ch4Active,
                int32_t gyroZCentiDps,
                int32_t absGyroCentiDps,
                int16_t effectiveSmoothingQ1000,
                int16_t effectiveDeadbandCentiDps,
                int16_t progressiveFactorQ1000,
                int16_t effectiveGainQ1000,
                int32_t correctionUsQ1000,
                int16_t driverPriorityFactorQ1000,
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
  printScaled(activeGyroGainQ1000, FIXED_SCALE, 2);
  Serial.print(F(" ch6="));
  if (damperValid) {
    Serial.print(damperPulseUs);
  } else {
    Serial.print(F("FAIL"));
  }
  Serial.print(F(" smooth="));
  printScaled(activeSmoothingQ1000, FIXED_SCALE, 2);
  Serial.print(F(" deadband="));
  printScaled(activeGyroDeadbandCentiDps, DPS_SCALE, 2);
  Serial.print(F(" ch4="));
  Serial.print(ch4Active ? F("ON") : F("OFF"));
  Serial.print(F(" recal="));
  Serial.print((uint8_t)recalState);
  Serial.print(F(" gyroZ="));
  printScaled(gyroZCentiDps, DPS_SCALE, 2);
  Serial.print(F(" absGyroZ="));
  printScaled(absGyroCentiDps, DPS_SCALE, 2);
  Serial.print(F(" effSmooth="));
  printScaled(effectiveSmoothingQ1000, FIXED_SCALE, 2);
  Serial.print(F(" effDeadband="));
  printScaled(effectiveDeadbandCentiDps, DPS_SCALE, 2);
  Serial.print(F(" prog="));
  printScaled(progressiveFactorQ1000, FIXED_SCALE, 2);
  Serial.print(F(" effGain="));
  printScaled(effectiveGainQ1000, FIXED_SCALE, 2);
  Serial.print(F(" corr="));
  printScaled(correctionUsQ1000, FIXED_SCALE, 1);
  Serial.print(F(" priority="));
  printScaled(driverPriorityFactorQ1000, FIXED_SCALE, 2);
  Serial.print(F(" out="));
  Serial.println(servoPulseUs);
}

void debugSafeControl(bool radioOk,
                      const ReceiverSnapshot &rx,
                      uint16_t steeringPulseUs) {
  debugPrint(radioOk,
             steeringPulseUs,
             rx.gain.pulseUs,
             rx.gainValid,
             rx.damper.pulseUs,
             rx.damperValid,
             recalButtonActive,
             0,
             0,
             activeSmoothingQ1000,
             activeGyroDeadbandCentiDps,
             PROGRESSIVE_GAIN_MIN_FACTOR_Q1000,
             0,
             0,
             FIXED_SCALE,
             SERVO_CENTER_US);
}

void runControlLoop(uint32_t nowUs, uint32_t nowMs) {
  ReceiverSnapshot rx = readReceiverSnapshot(nowUs);

  updateGainAndDamper(rx.gain, rx.gainValid, rx.damper, rx.damperValid);
  updateRecalibration(nowUs,
                      nowMs,
                      rx.recal,
                      rx.recalValid,
                      rx.steering,
                      rx.steeringValid);

  if (!rx.steeringValid) {
    setCenteredSafeOutput();
    if (DEBUG_SERIAL) {
      debugSafeControl(false, rx, SERVO_CENTER_US);
    }
    return;
  }

  uint16_t receiverPulseUs =
    clampPulse(rx.steering.pulseUs, RX_COMMAND_MIN_US, RX_COMMAND_MAX_US);

  if (recalibrationOwnsServo()) {
    setCenteredSafeOutput();
    if (DEBUG_SERIAL) {
      debugSafeControl(true, rx, receiverPulseUs);
    }
    return;
  }

  bool gyroOk = false;
  int32_t gyroZCentiDps =
    (mpuReady && initialGyroCalibrated) ? readGyroZCentiDps(gyroOk) : 0;

  if (!gyroOk) {
    gyroZCentiDps = 0;
  }

  ControlTerms terms = buildControlTerms(receiverPulseUs, gyroZCentiDps);
  int16_t gyroCorrectionUs = updateGyroCorrectionUs(terms);
  uint16_t servoPulseUs = buildServoCommand(receiverPulseUs, gyroCorrectionUs);
  setServoPulseUs(servoPulseUs);

  if (DEBUG_SERIAL) {
    debugPrint(true,
               receiverPulseUs,
               rx.gain.pulseUs,
               rx.gainValid,
               rx.damper.pulseUs,
               rx.damperValid,
               recalButtonActive,
               terms.gyroZCentiDps,
               terms.absGyroCentiDps,
               terms.effectiveSmoothingQ1000,
               terms.effectiveDeadbandCentiDps,
               terms.progressiveFactorQ1000,
               terms.effectiveGainQ1000,
               filteredCorrectionUsQ1000,
               terms.driverPriorityFactorQ1000,
               servoPulseUs);
  }
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

  if (!startMpu6050()) {
    mpuReady = false;
    initialGyroCalibrated = true;
  }

  if (DEBUG_SERIAL) {
    Serial.println(F("Firmware iniciado."));
  }
}

void loop() {
  uint32_t nowMs = millis();
  updateStatusLed(nowMs);

  uint32_t nowUs = micros();
  updateMpu6050Setup(nowMs, nowUs);
  updateInitialGyroCalibration(nowUs);

  if (!initialGyroCalibrated) {
    setServoPulseUs(SERVO_CENTER_US);
    return;
  }

  if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_INTERVAL_US) {
    lastControlUs = nowUs;
    runControlLoop(nowUs, nowMs);
  }
}
