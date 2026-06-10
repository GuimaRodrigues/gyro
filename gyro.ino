#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>

/*
  Firmware simples para gyro de drift RC RWD 1/10 com Arduino Uno/Nano e MPU6050.

  Ajustes de pista:
  - GYRO_GAIN_FIXED_US_PER_DPS: ganho principal. Aumente se o carro demora para
    contra-estercar. Reduza se a frente oscila ou o servo fica brigando.
  - GYRO_DEADBAND_DPS: zona morta do gyro. Aumente para ignorar vibracao/ruido.
    Reduza se o gyro ficar pouco sensivel no inicio do drift.
  - GYRO_SMOOTHING_ALPHA: suavizacao exponencial. 0.65-0.75 = mais rapido,
    0.85 = bom ponto inicial, 0.90-0.92 = mais suave, porem com mais atraso.
  - GYRO_CORRECTION_LIMIT_US: limite maximo da correcao em microssegundos.
    Aumente se falta contra-esterco. Reduza se o gyro domina demais a direcao.

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
const uint8_t RX_STEERING_PIN = 2;   // D2: interrupcao externa INT0
const uint8_t SERVO_OUTPUT_PIN = 9;  // D9: saida do servo gerada por Timer1
const uint8_t GAIN_POT_PIN = A0;     // Opcional: cursor do potenciometro
const uint8_t AUX_GAIN_PIN = 3;      // Opcional: canal AUX em D3/INT1

// ------------------------- Servo / receptor -------------------------
const uint16_t SERVO_MIN_US = 1000;
const uint16_t SERVO_CENTER_US = 1500;
const uint16_t SERVO_MAX_US = 2000;
const uint16_t SERVO_FRAME_US = 20000;       // 50 Hz
const uint32_t RX_SIGNAL_TIMEOUT_US = 100000UL;
const uint16_t RX_VALID_MIN_US = 900;
const uint16_t RX_VALID_MAX_US = 2100;

// Inverta se o servo estiver respondendo ao contrario do radio.
const bool SERVO_REVERSE = false;

// ------------------------- Gyro -------------------------
const uint8_t MPU6050_ADDRESS = 0x68;
const uint32_t I2C_CLOCK_HZ = 400000UL;
const uint8_t MPU6050_DLPF_CFG = 2;         // 2 = filtro interno ~94 Hz, baixa latencia
const float GYRO_LSB_PER_DPS = 65.5f;       // FS_SEL=1, faixa +/-500 graus/s
const uint16_t GYRO_CALIBRATION_SAMPLES = 1500;
const uint16_t GYRO_CALIBRATION_INTERVAL_US = 1000;
const uint16_t CONTROL_INTERVAL_US = 1000;  // controle a 1 kHz, servo recebe 200 Hz

// Inverta se o gyro corrigir para o lado errado.
const bool GYRO_REVERSE = false;

const float GYRO_GAIN_FIXED_US_PER_DPS = 1.5f;
const float GYRO_GAIN_MIN_US_PER_DPS = 0.0f;
const float GYRO_GAIN_MAX_US_PER_DPS = 8.0f;
const float GYRO_DEADBAND_DPS = 2.0f;
const float GYRO_SMOOTHING_ALPHA = 0.70f;
const int16_t GYRO_CORRECTION_LIMIT_US = 250;

// ------------------------- Ganho opcional -------------------------
enum GainMode {
  GAIN_FROM_FIXED,
  GAIN_FROM_POT,
  GAIN_FROM_AUX
};

// Use GAIN_FROM_FIXED para comecar. Depois troque para POT ou AUX se quiser
// ajustar o ganho na pista sem regravar o Arduino.
const GainMode GYRO_GAIN_MODE = GAIN_FROM_FIXED;
const uint16_t GAIN_UPDATE_INTERVAL_US = 10000; // 100 Hz para pot/AUX

// ------------------------- Debug -------------------------
#define DEBUG_SERIAL 0
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

struct PulseSnapshot {
  uint16_t pulseUs;
  uint32_t lastPulseUs;
  bool hasPulse;
};

volatile uint32_t steeringRiseUs = 0;
volatile uint16_t steeringPulseUs = SERVO_CENTER_US;
volatile uint32_t steeringLastPulseUs = 0;
volatile bool steeringHasPulse = false;

volatile uint32_t auxRiseUs = 0;
volatile uint16_t auxPulseUs = SERVO_CENTER_US;
volatile uint32_t auxLastPulseUs = 0;
volatile bool auxHasPulse = false;

volatile uint16_t servoPulseTicks = SERVO_CENTER_US * TIMER1_TICKS_PER_US;
volatile uint8_t *servoPort = 0;
uint8_t servoBitMask = 0;

bool mpuReady = false;
float gyroZOffsetRaw = 0.0f;
float filteredCorrectionUs = 0.0f;
float activeGyroGain = GYRO_GAIN_FIXED_US_PER_DPS;
uint32_t lastControlUs = 0;
uint32_t lastGainUpdateUs = 0;

uint16_t clampPulse(uint16_t value, uint16_t low, uint16_t high);
int16_t clampInt16(int16_t value, int16_t low, int16_t high);
float clampFloat(float value, float low, float high);
float mapPulseToGain(uint16_t pulseUs);
PulseSnapshot copySteeringPulse();
PulseSnapshot copyAuxPulse();
void steeringRxIsr();
void auxGainIsr();
void setupServoTimer200Hz();
void setServoPulseUs(uint16_t pulseUs);
bool writeMpuRegister(uint8_t reg, uint8_t value);
bool readMpuInt16(uint8_t reg, int16_t &value);
bool setupMpu6050();
void calibrateGyroZ();
float readGyroZDps(bool &ok);
void setupReceiverInputs();
void updateGyroGain(uint32_t nowUs);
uint16_t buildServoCommand(uint16_t receiverPulseUs, int16_t correctionUs);
void runControlLoop(uint32_t nowUs);

#if DEBUG_SERIAL
void debugPrint(bool radioOk,
                uint16_t receiverPulseUs,
                float gyroZDps,
                float gain,
                float correctionUs,
                uint16_t servoPulseUs);
#endif

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

float mapPulseToGain(uint16_t pulseUs) {
  uint16_t clipped = clampPulse(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
  float ratio = (float)(clipped - SERVO_MIN_US) / (float)(SERVO_MAX_US - SERVO_MIN_US);
  return GYRO_GAIN_MIN_US_PER_DPS +
         ratio * (GYRO_GAIN_MAX_US_PER_DPS - GYRO_GAIN_MIN_US_PER_DPS);
}

PulseSnapshot copySteeringPulse() {
  PulseSnapshot copy;
  uint8_t oldSREG = SREG;
  noInterrupts();
  copy.pulseUs = steeringPulseUs;
  copy.lastPulseUs = steeringLastPulseUs;
  copy.hasPulse = steeringHasPulse;
  SREG = oldSREG;
  return copy;
}

PulseSnapshot copyAuxPulse() {
  PulseSnapshot copy;
  uint8_t oldSREG = SREG;
  noInterrupts();
  copy.pulseUs = auxPulseUs;
  copy.lastPulseUs = auxLastPulseUs;
  copy.hasPulse = auxHasPulse;
  SREG = oldSREG;
  return copy;
}

void steeringRxIsr() {
  uint32_t now = micros();

  if (digitalRead(RX_STEERING_PIN) == HIGH) {
    steeringRiseUs = now;
    return;
  }

  uint32_t width = now - steeringRiseUs;
  if (width >= RX_VALID_MIN_US && width <= RX_VALID_MAX_US) {
    steeringPulseUs = (uint16_t)width;
    steeringLastPulseUs = now;
    steeringHasPulse = true;
  }
}

void auxGainIsr() {
  uint32_t now = micros();

  if (digitalRead(AUX_GAIN_PIN) == HIGH) {
    auxRiseUs = now;
    return;
  }

  uint32_t width = now - auxRiseUs;
  if (width >= RX_VALID_MIN_US && width <= RX_VALID_MAX_US) {
    auxPulseUs = (uint16_t)width;
    auxLastPulseUs = now;
    auxHasPulse = true;
  }
}

ISR(TIMER1_COMPA_vect) {
  *servoPort |= servoBitMask;
  OCR1B = servoPulseTicks;
}

ISR(TIMER1_COMPB_vect) {
  *servoPort &= (uint8_t)~servoBitMask;
}

void setupServoTimer200Hz() {
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
  uint16_t safePulse = clampPulse(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
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
  ok &= writeMpuRegister(MPU_REG_PWR_MGMT_1, 0x01);  // acorda e usa clock do gyro X
  delay(100);
  ok &= writeMpuRegister(MPU_REG_CONFIG, MPU6050_DLPF_CFG);
  ok &= writeMpuRegister(MPU_REG_SMPLRT_DIV, 0x00);  // 1 kHz com DLPF ativo
  ok &= writeMpuRegister(MPU_REG_GYRO_CONFIG, 0x08);  // FS_SEL=1, +/-500 dps
  return ok;
}

void calibrateGyroZ() {
  int32_t sum = 0;
  uint16_t goodSamples = 0;
  int16_t rawZ = 0;

#if DEBUG_SERIAL
  Serial.println(F("Calibrando gyro Z. Deixe o carro parado..."));
#endif

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

#if DEBUG_SERIAL
  Serial.print(F("Offset gyro Z raw = "));
  Serial.println(gyroZOffsetRaw);
#endif
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

  int steeringInterrupt = digitalPinToInterrupt(RX_STEERING_PIN);
  if (steeringInterrupt != NOT_AN_INTERRUPT) {
    attachInterrupt(steeringInterrupt, steeringRxIsr, CHANGE);
  }

  if (GYRO_GAIN_MODE == GAIN_FROM_AUX) {
    pinMode(AUX_GAIN_PIN, INPUT);
    int auxInterrupt = digitalPinToInterrupt(AUX_GAIN_PIN);
    if (auxInterrupt != NOT_AN_INTERRUPT) {
      attachInterrupt(auxInterrupt, auxGainIsr, CHANGE);
    }
  }
}

void updateGyroGain(uint32_t nowUs) {
  if ((uint32_t)(nowUs - lastGainUpdateUs) < GAIN_UPDATE_INTERVAL_US) {
    return;
  }

  lastGainUpdateUs = nowUs;

  if (GYRO_GAIN_MODE == GAIN_FROM_POT) {
    int raw = analogRead(GAIN_POT_PIN);
    float ratio = (float)raw / 1023.0f;
    activeGyroGain = GYRO_GAIN_MIN_US_PER_DPS +
                     ratio * (GYRO_GAIN_MAX_US_PER_DPS - GYRO_GAIN_MIN_US_PER_DPS);
    return;
  }

  if (GYRO_GAIN_MODE == GAIN_FROM_AUX) {
    PulseSnapshot aux = copyAuxPulse();
    if (aux.hasPulse && (uint32_t)(nowUs - aux.lastPulseUs) <= RX_SIGNAL_TIMEOUT_US) {
      activeGyroGain = mapPulseToGain(aux.pulseUs);
      return;
    }
  }

  activeGyroGain = GYRO_GAIN_FIXED_US_PER_DPS;
}

uint16_t buildServoCommand(uint16_t receiverPulseUs, int16_t correctionUs) {
  int16_t steeringDeltaUs = (int16_t)receiverPulseUs - (int16_t)SERVO_CENTER_US;
  int16_t outputDeltaUs = steeringDeltaUs + correctionUs;

  if (SERVO_REVERSE) {
    outputDeltaUs = -outputDeltaUs;
  }

  int16_t outputUs = (int16_t)SERVO_CENTER_US + outputDeltaUs;
  return clampPulse((uint16_t)clampInt16(outputUs, SERVO_MIN_US, SERVO_MAX_US),
                    SERVO_MIN_US,
                    SERVO_MAX_US);
}

#if DEBUG_SERIAL
void debugPrint(bool radioOk,
                uint16_t receiverPulseUs,
                float gyroZDps,
                float gain,
                float correctionUs,
                uint16_t servoPulseUs) {
  static uint32_t lastDebugMs = 0;
  uint32_t nowMs = millis();

  if ((uint32_t)(nowMs - lastDebugMs) < DEBUG_INTERVAL_MS) {
    return;
  }

  lastDebugMs = nowMs;

  Serial.print(F("radio="));
  Serial.print(radioOk ? F("OK") : F("FAIL"));
  Serial.print(F(" rx="));
  Serial.print(receiverPulseUs);
  Serial.print(F(" gyroZ="));
  Serial.print(gyroZDps, 2);
  Serial.print(F(" gain="));
  Serial.print(gain, 2);
  Serial.print(F(" corr="));
  Serial.print(correctionUs, 1);
  Serial.print(F(" out="));
  Serial.println(servoPulseUs);
}
#endif

void runControlLoop(uint32_t nowUs) {
  updateGyroGain(nowUs);

  PulseSnapshot steering = copySteeringPulse();
  bool radioOk = steering.hasPulse &&
                 (uint32_t)(nowUs - steering.lastPulseUs) <= RX_SIGNAL_TIMEOUT_US;

  if (!radioOk) {
    filteredCorrectionUs = 0.0f;
    setServoPulseUs(SERVO_CENTER_US);

#if DEBUG_SERIAL
    debugPrint(false, SERVO_CENTER_US, 0.0f, activeGyroGain, 0.0f, SERVO_CENTER_US);
#endif
    return;
  }

  uint16_t receiverPulseUs = clampPulse(steering.pulseUs, SERVO_MIN_US, SERVO_MAX_US);

  bool gyroOk = false;
  float gyroZDps = mpuReady ? readGyroZDps(gyroOk) : 0.0f;

  if (!gyroOk) {
    gyroZDps = 0.0f;
  }

  if (gyroZDps > -GYRO_DEADBAND_DPS && gyroZDps < GYRO_DEADBAND_DPS) {
    gyroZDps = 0.0f;
  }

  float correctionUs = gyroZDps * activeGyroGain;
  if (GYRO_REVERSE) {
    correctionUs = -correctionUs;
  }

  correctionUs = clampFloat(correctionUs,
                            -(float)GYRO_CORRECTION_LIMIT_US,
                            (float)GYRO_CORRECTION_LIMIT_US);

  filteredCorrectionUs =
    filteredCorrectionUs * GYRO_SMOOTHING_ALPHA +
    correctionUs * (1.0f - GYRO_SMOOTHING_ALPHA);

  filteredCorrectionUs = clampFloat(filteredCorrectionUs,
                                    -(float)GYRO_CORRECTION_LIMIT_US,
                                    (float)GYRO_CORRECTION_LIMIT_US);

  uint16_t servoPulseUs = buildServoCommand(receiverPulseUs, (int16_t)filteredCorrectionUs);
  setServoPulseUs(servoPulseUs);

#if DEBUG_SERIAL
  debugPrint(true,
             receiverPulseUs,
             gyroZDps,
             activeGyroGain,
             filteredCorrectionUs,
             servoPulseUs);
#endif
}

void setup() {
#if DEBUG_SERIAL
  Serial.begin(DEBUG_BAUD);
#endif

  setupServoTimer200Hz();
  setServoPulseUs(SERVO_CENTER_US);
  setupReceiverInputs();

  mpuReady = setupMpu6050();
  if (mpuReady) {
    calibrateGyroZ();
  }

#if DEBUG_SERIAL
  Serial.print(F("MPU6050: "));
  Serial.println(mpuReady ? F("OK") : F("FALHA"));
  Serial.println(F("Firmware pronto."));
#endif
}

void loop() {
  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_INTERVAL_US) {
    lastControlUs = nowUs;
    runControlLoop(nowUs);
  }
}
