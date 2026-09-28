#include <Wire.h>

#define STEP_LEFT     6   // PD6
#define DIR_LEFT      7   // PD7
#define STEP_RIGHT    8   // PB0
#define DIR_RIGHT     9   // PB1

#define STEP_L_BIT    (1 << PD6)
#define STEP_R_BIT    (1 << PB0)
#define DIR_L_BIT     (1 << PD7)
#define DIR_R_BIT     (1 << PB1)

float Kp = 65.0;
float Ki = 0.4;
float Kd = 1.0;

float TARGET_ANGLE = 5.6;

#define LOOP_HZ           250
#define LOOP_US           (1000000UL / LOOP_HZ)

#define COMP_ALPHA        0.98f

#define D_LPF_ALPHA       0.30f

#define MAX_SPEED          2500.0f
#define MAX_ACCEL          60000.0f
#define SPEED_TO_DELAY_K   100000.0f
#define MIN_STEP_PERIOD_US 40
#define MAX_STEP_PERIOD_US 20000
#define I_LIMIT        25.0f

#define FALL_LIMIT        35.0f

#define AUTO_TRIM_GAIN         0.0f
#define TRIM_LIMIT             7.0f

#define DEBUG_SERIAL      1
#define DEBUG_EVERY_N     25                 // co ile petli drukowac

float compAngle     = 0.0f;
float gyroBiasX     = 0.0f;
float errorIntegral = 0.0f;
float dTermFilt     = 0.0f;
float cmdSpeed      = 0.0f;           // aktualna (wygladzona) predkosc komendy
float angleTrim     = 0.0f;           // korekta setpointu z auto-trim

volatile bool motorsRunning = false;

unsigned long lastLoopUs = 0;
uint16_t dbgCounter = 0;

ISR(TIMER1_COMPA_vect) {
  if (motorsRunning) {
    PORTD |= STEP_L_BIT;
    PORTB |= STEP_R_BIT;
    // impuls > 100 ns (wymog TMC2226)
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    PORTD &= ~STEP_L_BIT;
    PORTB &= ~STEP_R_BIT;
  }
}

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuInit() {
  mpuWrite(0x6B, 0x00);       // PWR_MGMT_1 - wybudz
  delay(10);
  mpuWrite(0x1A, 0x03);       // CONFIG/DLPF=3 -> zyro 42 Hz / akcel 44 Hz (kluczowe!)
  mpuWrite(0x1B, 0x00);       // GYRO_CONFIG -> +-250 st/s (131 LSB/(st/s))
  mpuWrite(0x1C, 0x00);       // ACCEL_CONFIG -> +-2 g
  delay(10);
}

void calibrateGyro() {
  long sum = 0;
  for (int i = 0; i < 1000; i++) {        // ~2 s - TRZYMAJ ROBOTA NIERUCHOMO
    Wire.beginTransmission(0x68);
    Wire.write(0x43);                     // GYRO_XOUT_H
    Wire.endTransmission();
    Wire.requestFrom(0x68, 2);
    sum += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(2);
  }
  gyroBiasX = sum / 1000.0f;
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  pinMode(DIR_LEFT, OUTPUT);
  pinMode(STEP_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  pinMode(STEP_RIGHT, OUTPUT);

  // Timer1: CTC, prescaler 8 (2 MHz -> 0.5 us/tik)
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 65535;
  TCCR1B |= (1 << WGM12);   // CTC
  TCCR1B |= (1 << CS11);    // prescaler 8
  TIMSK1 |= (1 << OCIE1A); // przerwanie przy zrownaniu
  sei();

  mpuInit();
  delay(100);
  calibrateGyro();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t ax = Wire.read() << 8 | Wire.read(); (void)ax;
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  compAngle = atan2((float)ay, (float)az) * 180.0f / PI;

  lastLoopUs = micros();
}

void loop() {
  unsigned long now = micros();
  if ((now - lastLoopUs) < LOOP_US) return;        // czekaj na slot petli
  float dt = (now - lastLoopUs) * 1e-6f;
  lastLoopUs = now;

  // --- Odczyt IMU (burst 14 bajtow jednym strzalem) ---
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  int16_t accX = Wire.read() << 8 | Wire.read(); (void)accX;
  int16_t accY = Wire.read() << 8 | Wire.read();
  int16_t accZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();                       // temperatura - pomijamy
  int16_t gyroXr = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();                        // gyroY - pomijamy
  Wire.read(); Wire.read();                        // gyroZ - pomijamy

  float gyroX = (gyroXr - gyroBiasX) / 131.0f;    // [st/s]

  // --- Filtr komplementarny ---
  float accAngle = atan2((float)accY, (float)accZ) * 180.0f / PI;
  compAngle = COMP_ALPHA * (compAngle + gyroX * dt) + (1.0f - COMP_ALPHA) * accAngle;

  // --- Setpoint (z ewentualna korekta auto-trim) i blad ---
  float setpoint = TARGET_ANGLE + angleTrim;
  float error = setpoint - compAngle;

  // --- Bezpieczenstwo: upadek -> stop i reset integratorow ---
  if (fabs(error) > FALL_LIMIT) {
    motorsRunning = false;
    errorIntegral = 0.0f;
    cmdSpeed = 0.0f;
    // angleTrim zostawiamy - to wyznaczony punkt rownowagi
    return;
  }

  // --- PID ---
  errorIntegral += error * dt;
  errorIntegral = constrain(errorIntegral, -I_LIMIT, I_LIMIT);

  // czlon D na pomiarze (predkosc katowa), filtrowany dolnoprzepustowo
  float dRaw = -gyroX;
  dTermFilt += D_LPF_ALPHA * (dRaw - dTermFilt);

  float output = Kp * error + Ki * errorIntegral + Kd * dTermFilt;

  // --- Wygladzenie komendy predkosci (slew-rate limiter) ---
  output = constrain(output, -MAX_SPEED, MAX_SPEED);
  float maxDelta = MAX_ACCEL * dt;
  float delta = constrain(output - cmdSpeed, -maxDelta, maxDelta);
  cmdSpeed += delta;

  // --- Auto-trim punktu rownowagi (wolna petla, opcjonalna) ---
  if (AUTO_TRIM_GAIN > 0.0f) {
    angleTrim += AUTO_TRIM_GAIN * cmdSpeed * dt;
    angleTrim = constrain(angleTrim, -TRIM_LIMIT, TRIM_LIMIT);
  }

  // --- Wyslij do silnikow ---
  setMotorSpeed(cmdSpeed);

  // --- Debug ---
  #if DEBUG_SERIAL
  if (++dbgCounter >= DEBUG_EVERY_N) {
    dbgCounter = 0;
    Serial.print("ang="); Serial.print(compAngle, 2);
    Serial.print(" err="); Serial.print(error, 2);
    Serial.print(" out="); Serial.print(output, 0);
    Serial.print(" spd="); Serial.print(cmdSpeed, 0);
    Serial.print(" trim=");Serial.println(angleTrim, 2);
  }
  #endif
}

void setMotorSpeed(float speed) {
  // kierunek (DIR jednego silnika odwrocony - lustrzane zamontowanie)
  if (speed >= 0) {
    PORTD |= DIR_L_BIT;
    PORTB &= ~DIR_R_BIT;
  } else {
    PORTD &= ~DIR_L_BIT;
    PORTB |= DIR_R_BIT;
  }

  float mag = fabs(speed);

  if (mag < 1.0f) {           // praktycznie zero -> trzymaj pradem, nie krokuj
    motorsRunning = false;
    return;
  }

  // predkosc -> okres kroku [us]
  float periodUs = SPEED_TO_DELAY_K / mag;
  if (periodUs < MIN_STEP_PERIOD_US) periodUs = MIN_STEP_PERIOD_US;
  if (periodUs > MAX_STEP_PERIOD_US) periodUs = MAX_STEP_PERIOD_US;

  // okres[us] -> tiki timera (0.5 us/tik => *2)
  uint16_t newOCR = (uint16_t)(periodUs * 2.0f);

  cli();
  OCR1A = newOCR;
  if (TCNT1 > newOCR) TCNT1 = 0;   // unik glitcha zawiniecia licznika przy przyspieszaniu
  sei();

  motorsRunning = true;
}
