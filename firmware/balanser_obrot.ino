// =====================================================================
// Robot samobalansujacy - WARIANT Z AUTOMATYCZNYM OBROTEM (v2)
// Co TURN_INTERVAL_MS robot wykonuje obrot w lewo o TURN_ANGLE_DEG,
// nie przerywajac balansowania.
//
// Krokowanie obu silnikow: jedno LEKKIE przerwanie Timer1 o stalej,
// UMIARKOWANEJ czestotliwosci (STEP_ISR_HZ) z dwoma akumulatorami fazy
// (DDS). Arytmetyka 16-bitowa, kierunek ustawiany POZA przerwaniem -
// dzieki czemu ISR jest bardzo krotki i nie zaglusza petli/I2C.
// Predkosc kazdego kola = skladowa balansu +/- skladowa obrotu.
//
// UWAGA: w razie braku obrotu/zlej strony - zmienic znak TURN_DIR;
// predkosc obrotu reguluje TURN_RATE. Plyta w Arduino IDE: "Arduino Uno"
// (ATmega328P, 16 MHz).
// =====================================================================
#include <Wire.h>

#define STEP_LEFT    6   // PD6
#define DIR_LEFT     7   // PD7
#define STEP_RIGHT   8   // PB0
#define DIR_RIGHT    9   // PB1

#define STEP_L_BIT   (1 << PD6)
#define STEP_R_BIT   (1 << PB0)
#define DIR_L_BIT    (1 << PD7)
#define DIR_R_BIT    (1 << PB1)

// --- regulator (jak w wersji podstawowej) ---
float Kp = 65.0;
float Ki = 0.4;
float Kd = 1.0;
float TARGET_ANGLE = 5.6;      // bazowy kat rownowagi (moze byc nadpisany auto-kalibracja)
float angleTrim    = 0.0f;     // powolna korekta setpointu (auto-trim)

// === AUTO-KALIBRACJA KATA ROWNOWAGI ===
// Punkt rownowagi zalezy od rozkladu masy (np. bateria 9V na gorze go przesuwa), wiec
// sztywne 5,6 stopnia bywa zle -> robot ma stale przechylenie i "odjezdza".
// AUTO_TARGET_AT_START: po kalibracji zyroskopu przyjmij kat, pod ktorym TRZYMANO robota,
//   jako punkt rownowagi. -> Trzymaj robota DOKLADNIE w pionie/rownowadze przez te ~2 s.
// AUTO_TRIM: powolne dostrajanie setpointu w trakcie pracy, by srednia predkosc kol = 0
//   (robot przestaje "odjezdzac"). UWAGA: gdyby zaczal narastajaco odjezdzac - zmien znak.
// WAZNE: AUTO_TARGET domyslnie WYLACZONE. Przechwytywalo kat, pod ktorym TRZYMANO robota,
// a to rzadko jest dokladny punkt rownowagi -> robot nie balansowal i sie wywracal.
// PEWNY sposob ustawienia rownowagi (recznie): postaw/przytrzymaj robota dokladnie w
// rownowadze, odczytaj 'ang=' z monitora szeregowego (115200) i wpisz te wartosc do
// TARGET_ANGLE powyzej. Dopiero gdy to dziala - mozna ewentualnie sprobowac AUTO_TARGET=1
// (trzymajac robota PRECYZYJNIE w punkcie rownowagi przez czas kalibracji).
#define AUTO_TARGET_AT_START 0         // opcjonalne, domyslnie WYL
#define AUTO_TRIM            0         // opcjonalne, domyslnie WYL (zly znak -> narastajacy dryf)
#define AUTO_TRIM_GAIN       0.0006f   // male! dostrajanie jest wolne
#define TRIM_LIMIT           8.0f      // ograniczenie korekty [stopnie]

#define LOOP_HZ          250
#define LOOP_US          (1000000UL / LOOP_HZ)
#define COMP_ALPHA       0.98f
#define D_LPF_ALPHA      0.30f
#define DEBUG_SERIAL     1               // 0 = wylacz telemetrie (zero narzutu na float->tekst)
#define DEBUG_EVERY_N    25              // co ile petli drukowac (25 => ~10 Hz)

// Budzet CPU (ATmega328P @16 MHz): ISR krokowania 25 kHz, lekki (16-bit) ~ 8%;
// odczyt I2C 14 bajtow @400 kHz ~ 10%; obliczenia petli 250 Hz (1x atan2 + float) < 10%.
// Zapas > 70% - procesor nie jest obciazony. (Telemetria float jest najciezsza czescia
// opcjonalna - mozna ja wylaczyc DEBUG_SERIAL=0.)

#define MAX_SPEED           2500.0f    // jak w oryginale -> rate <= 25000
#define MAX_ACCEL           60000.0f
#define SPEED_TO_RATE       10.0f      // cmdSpeed -> mikrokroki/s
#define MAX_RATE            25000      // ograniczenie czestotliwosci krokow [1/s]
#define RATE_DEADBAND       8          // ponizej tej predkosci nie krokuj (jak oryginal)
#define I_LIMIT         25.0f
#define FALL_LIMIT      35.0f

// --- krokowanie DDS (lekki ISR) ---
// Timer1 CTC, presk. 8 (2 MHz) -> OCR1A = 79 -> 25 000 Hz
#define STEP_ISR_HZ        25000U
volatile uint16_t rateMagL = 0;        // |predkosc| lewego [mikrokroki/s]
volatile uint16_t rateMagR = 0;        // |predkosc| prawego
volatile uint16_t accL = 0, accR = 0; // akumulatory fazy
volatile bool     motorsEnabled = false;

// --- parametry obrotu ---
#define MICROSTEPS_PER_REV 6400.0f      // 200 krokow * 1/32 mikrokroku
#define WHEEL_DIAM_CM      7.0f
#define TRACK_CM           15.5f        // rozstaw kol
#define TURN_ANGLE_DEG     45.0f         // mniejszy obrot = mniejsze zaburzenie, latwiej utrzymac pion
#define TURN_INTERVAL_MS   13333UL       // ~co 13 s (1,5x czesciej niz 20 s)
#define TURN_RATE          1200.0f      // roznicowa predkosc obrotu [mikrokroki/s] (wolniej = pewniej)
#define TURN_RAMP          3000.0f      // przyspieszenie obrotu [mikrokroki/s^2] (lagodny start)
#define TURN_DIR           (+1.0f)      // +1 obrot w lewo (ew. zmienic na -1)
// PRIORYTET BALANSU - PROPORCJONALNY (zamiast binarnego, ktory blokowal obrot na stale):
//   |error| < TURN_PITCH_FULL    -> pelna predkosc obrotu,
//   FULL..STOP                   -> predkosc obrotu LINIOWO maleje do 0 (przy chwilowym
//                                   przechyle obrot tylko zwalnia i mimo to sie konczy),
//   |error| >= TURN_PITCH_EMERG -> NATYCHMIASTOWE odciecie (awaryjne, duzy przechyl).
// Mniejsze progi = obrot zwalnia juz przy malym przechyle -> robot trzyma sie blizej PIONU
// podczas obrotu (mniej "wahania"), kosztem dluzszego czasu obrotu.
#define TURN_PITCH_FULL    1.5f         // [stopnie] ponizej -> pelny obrot
#define TURN_PITCH_STOP    4.0f         // [stopnie] przy tym przechyle predkosc obrotu = 0
#define TURN_PITCH_EMERG   7.0f         // [stopnie] awaryjne natychmiastowe odciecie
// PRZYGOTOWANIE DO OBROTU: po uplywie interwalu obrot jest "uzbrajany", ale startuje
// dopiero, gdy robot jest pionowo i spokojny przez TURN_READY_TIME (wejscie w obrot
// z dobrego stanu = duzo pewniej, zwlaszcza przy ciezkiej gorze).
#define TURN_READY_ANGLE   1.5f         // |error| ponizej -> uznaj za pionowo [stopnie]
#define TURN_READY_RATE    40.0f        // |predkosc katowa pochylenia| ponizej [stopnie/s]
#define TURN_READY_SPEED   120.0f       // |cmdSpeed| ponizej -> robot nie jedzie/spokojny
#define TURN_READY_TIME    0.30f        // wymagany czas ciaglej stabilnosci [s]
#define TURN_ARM_TIMEOUT_MS 4000UL      // gdy nie wyciszy sie w tym czasie - obroc mimo to

// --- opcjonalne utrzymanie kursu (yaw) = rozna predkosc kol w pętli balansu ---
// Wykorzystuje zyroskop w osi Z (dotad pomijany). Korekta jest ROZNICOWA, wiec nie
// odbiera autorytetu balansowi (wspolnemu). Efekt: robot nie znosi/nie obraca sie sam,
// trzyma kurs i opiera sie probie skrecenia. 1 = wlacz, 0 = wylacz.
#define ENABLE_YAW_HOLD    0
#define KYAW_P             40.0f       // wzmocnienie od bledu kursu [stopnie]
#define KYAW_D             8.0f        // wzmocnienie od predkosci katowej yaw [stopnie/s]
#define YAW_CORR_LIMIT     1500.0f     // ograniczenie korekty roznicowej [mikrokroki/s]

// --- opcjonalne sterowanie pozycja / jazda przod-tyl ---
// Silnik krokowy = znana predkosc, wiec calka cmdSpeed daje pozycje (darmowa odometria).
// Petla ZEWNETRZNA lekko pochyla setpoint, by trzymac pozycje (anti-drift, driveCmd=0)
// albo jechac z zadana predkoscia (driveCmd != 0). Jazda na balansie = sterowanie
// pochyleniem, NIE bezposrednio predkoscia kol. 1 = wlacz, 0 = wylacz.
#define ENABLE_POS_HOLD    0
#define KPOS_P             0.0006f     // od bledu pozycji [stopnie/krok]
#define KPOS_D             0.004f      // od predkosci [stopnie/(krok/s)]
#define SETPOINT_OFFSET_LIMIT 3.0f     // max pochylenie zadane przez petle pozycji [stopnie]
float   driveCmd = 0.0f;               // zadana predkosc jazdy [mikrokroki/s]; 0=stoj, >0=przod
float   posEst   = 0.0f;               // estymata pozycji [mikrokroki]

float compAngle     = 0.0f;
float gyroBiasX     = 0.0f;
float gyroBiasZ     = 0.0f;    // offset zyroskopu osi Z (yaw)
float heading       = 0.0f;    // scalony kurs [stopnie]
float errorIntegral = 0.0f;
float dTermFilt     = 0.0f;
float cmdSpeed      = 0.0f;

unsigned long lastLoopUs = 0;
uint32_t      lastTurnMs = 0;
float         turnStepsLeft = 0.0f;
float         turnCmd = 0.0f;          // biezaca (wygladzona) predkosc obrotu
bool          turning = false;
bool          turnArmed = false;       // faza przygotowania - czeka na wyciszenie
float         settleTimer = 0.0f;      // czas ciaglej stabilnosci [s]
uint32_t      armMs = 0;               // moment uzbrojenia (do timeoutu)
uint16_t      dbgCounter = 0;

// liczba mikrokrokow na kolo dla obrotu o TURN_ANGLE_DEG:
const float TURN_STEPS =
(TURN_ANGLE_DEG / 180.0f) * (TRACK_CM * 0.5f) / WHEEL_DIAM_CM * MICROSTEPS_PER_REV;

// --- ISR krokowania: krotki, 16-bitowy, bez galezi na znak ---
ISR(TIMER1_COMPA_vect) {
  if (!motorsEnabled) return;
  accL += rateMagL;
  if (accL >= STEP_ISR_HZ) {
    accL -= STEP_ISR_HZ;
    PORTD |= STEP_L_BIT;
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    PORTD &= ~STEP_L_BIT;
  }
  accR += rateMagR;
  if (accR >= STEP_ISR_HZ) {
    accR -= STEP_ISR_HZ;
    PORTB |= STEP_R_BIT;
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    PORTB &= ~STEP_R_BIT;
  }
}

// Odblokowanie ZAWIESZONEJ magistrali I2C (po upadku/zaniku zasilania MPU czujnik
// potrafi trzymac SDA nisko; sam reset MCU tego nie naprawia). Wyklocz do 9 impulsow
// SCL, az slave zwolni SDA, potem wygeneruj warunek STOP. SDA=A4, SCL=A5.
void i2cRecover() {
  pinMode(A4, INPUT_PULLUP);   // SDA
  pinMode(A5, INPUT_PULLUP);   // SCL
  delayMicroseconds(10);
  for (uint8_t i = 0; i < 9 && digitalRead(A4) == LOW; i++) {
    pinMode(A5, OUTPUT); digitalWrite(A5, LOW); delayMicroseconds(5); // SCL low
    pinMode(A5, INPUT_PULLUP);                  delayMicroseconds(5); // SCL high (pullup)
  }
  // warunek STOP: SDA z LOW na HIGH przy SCL HIGH
  pinMode(A4, OUTPUT); digitalWrite(A4, LOW); delayMicroseconds(5);
  pinMode(A5, INPUT_PULLUP);                   delayMicroseconds(5);
  pinMode(A4, INPUT_PULLUP);                   delayMicroseconds(5);
}

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuInit() {
  mpuWrite(0x6B, 0x80);   // DEVICE_RESET - przywroc czujnik do stanu znanego (wazne po upadku)
  delay(100);
  mpuWrite(0x6B, 0x00);   // PWR_MGMT_1 - wybudz (skasuj sleep)
  delay(10);
  mpuWrite(0x1A, 0x03);   // DLPF=3 -> zyro 42 Hz
  mpuWrite(0x1B, 0x00);   // GYRO_CONFIG +-250 st/s (131 LSB/(st/s))
  mpuWrite(0x1C, 0x00);   // ACCEL_CONFIG +-2 g
  delay(10);
}

void calibrateGyro() {
  long sumX = 0, sumZ = 0;
  for (int i = 0; i < 1000; i++) {          // ~2 s - TRZYMAJ ROBOTA NIERUCHOMO
    Wire.beginTransmission(0x68);
    Wire.write(0x43);                       // GYRO_XOUT_H .. ZOUT_L
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t gx = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();             // gyroY - pomijamy
    int16_t gz = Wire.read() << 8 | Wire.read();
    sumX += gx; sumZ += gz;
    delay(2);
  }
  gyroBiasX = sumX / 1000.0f;
  gyroBiasZ = sumZ / 1000.0f;
}

// ustaw kierunki (DIR) poza ISR i zapisz magnitudy predkosci kol
void setMotorRates(float rL, float rR) {
  if (rL >= 0) PORTD |= DIR_L_BIT; else PORTD &= ~DIR_L_BIT;
  if (rR >= 0) PORTB &= ~DIR_R_BIT; else PORTB |= DIR_R_BIT;    // prawy odwrocony

  float mL = fabs(rL); if (mL > MAX_RATE) mL = MAX_RATE;
  float mR = fabs(rR); if (mR > MAX_RATE) mR = MAX_RATE;
  if (mL < RATE_DEADBAND) mL = 0;       // brak chaotycznego krokowania przy ~zerowej predkosci
  if (mR < RATE_DEADBAND) mR = 0;

  uint16_t iL = (uint16_t)mL;
  uint16_t iR = (uint16_t)mR;
  cli();
  rateMagL = iL;
  rateMagR = iR;
  sei();
}

void setup() {
  i2cRecover();              // odblokuj ewentualnie zawieszona magistrale I2C (po upadku)
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  pinMode(DIR_LEFT, OUTPUT);
  pinMode(STEP_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  pinMode(STEP_RIGHT, OUTPUT);

  // Timer1: CTC, presk. 8 (2 MHz). OCR1A=79 -> (79+1) tikow -> 25 000 Hz
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 79;
  TCCR1B |= (1 << WGM12);   // CTC
  TCCR1B |= (1 << CS11);    // presk. 8
  TIMSK1 |= (1 << OCIE1A);
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

  #if AUTO_TARGET_AT_START
  TARGET_ANGLE = compAngle;        // kat, pod ktorym trzymano robota podczas kalibracji = rownowaga
  #endif

  lastLoopUs = micros();
  lastTurnMs = millis();
}

void loop() {
  unsigned long now = micros();
  if ((now - lastLoopUs) < LOOP_US) return;
  float dt = (now - lastLoopUs) * 1e-6f;
  lastLoopUs = now;

  // --- Odczyt IMU (burst 14 bajtow) ---
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  int16_t accX = Wire.read() << 8 | Wire.read(); (void)accX;
  int16_t accY = Wire.read() << 8 | Wire.read();
  int16_t accZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();                       // temperatura
  int16_t gyroXr = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();                        // gyroY - pomijamy
  int16_t gyroZr = Wire.read() << 8 | Wire.read(); // gyroZ - kurs (yaw)

  float gyroX = (gyroXr - gyroBiasX) / 131.0f;
  float yawRate = (gyroZr - gyroBiasZ) / 131.0f;    // [stopnie/s]

  // --- Filtr komplementarny ---
  float accAngle = atan2((float)accY, (float)accZ) * 180.0f / PI;
  compAngle = COMP_ALPHA * (compAngle + gyroX * dt) + (1.0f - COMP_ALPHA) * accAngle;

  // --- Petla zewnetrzna: utrzymanie pozycji / jazda przod-tyl (opcjonalna) ---
  // Aktywna tylko gdy wlaczona globalnie (ENABLE_POS_HOLD). NIE jest wlaczana podczas
  // obrotu - obrot pozostaje "czysty" (sam balans + skladowa roznicowa), co jest
  // maksymalnie odporne na wywrocenie. Obrot i tak jest praktycznie w miejscu, bo faza
  // przygotowania uruchamia go dopiero, gdy robot stoi (pelzanie znikome).
  bool posHoldActive = ENABLE_POS_HOLD;
  static bool prevPosHold = false;
  if (posHoldActive && !prevPosHold) posEst = 0.0f; // zlap biezaca pozycje na starcie trzymania
  prevPosHold = posHoldActive;

  float setpoint = TARGET_ANGLE + angleTrim;               // baza + powolna korekta (auto-trim)
  if (posHoldActive) {
    float dcmd = (turning || turnArmed) ? 0.0f : driveCmd; // w obrocie zawsze trzymaj pozycje
    float vel = cmdSpeed * SPEED_TO_RATE - dcmd;            // odchylka predkosci od zadanej
    posEst += vel * dt;                                     // calka -> blad pozycji [kroki]
    float spOff = -(KPOS_P * posEst + KPOS_D * vel);        // pochyl, by wrocic do pozycji/predkosci
    spOff = constrain(spOff, -SETPOINT_OFFSET_LIMIT, SETPOINT_OFFSET_LIMIT);
    setpoint += spOff;                                      // jazda/utrzymanie = celowe pochylenie
  }
  float error = setpoint - compAngle;

  // --- Bezpieczenstwo: upadek ---
  if (fabs(error) > FALL_LIMIT) {
    motorsEnabled = false;
    errorIntegral = 0.0f;
    cmdSpeed = 0.0f;
    turning = false;
    turnArmed = false;
    turnCmd = 0.0f;
    posEst = 0.0f;
    setMotorRates(0, 0);
    lastTurnMs = millis();          // odlicz obrot od nowa po podniesieniu
    return;
  }
  motorsEnabled = true;

  // --- PID ---
  errorIntegral += error * dt;
  errorIntegral = constrain(errorIntegral, -I_LIMIT, I_LIMIT);
  float dRaw = -gyroX;
  dTermFilt += D_LPF_ALPHA * (dRaw - dTermFilt);
  float output = Kp * error + Ki * errorIntegral + Kd * dTermFilt;

  // --- Wygladzenie komendy predkosci (balans) ---
  output = constrain(output, -MAX_SPEED, MAX_SPEED);
  float maxDelta = MAX_ACCEL * dt;
  float delta = constrain(output - cmdSpeed, -maxDelta, maxDelta);
  cmdSpeed += delta;

  // --- AUTO-TRIM: powolne dostrajanie punktu rownowagi ---
  // Jezeli robot ciagle jedzie w jedna strone (cmdSpeed != 0), znaczy ze setpoint jest
  // lekko przesuniety -> powoli koryguj, az srednia predkosc kol bedzie ~0 (robot stoi).
  // Wstrzymane podczas obrotu/jazdy, by ich nie "rozregulowac".
  #if AUTO_TRIM
  if (!turning && !turnArmed) {
    angleTrim += AUTO_TRIM_GAIN * cmdSpeed * dt;
    angleTrim = constrain(angleTrim, -TRIM_LIMIT, TRIM_LIMIT);
  }
  #endif

  // --- Harmonogram obrotu: UZBROJENIE -> PRZYGOTOWANIE -> OBROT ---
  uint32_t nowMs = millis();
  // 1) co TURN_INTERVAL_MS uzbrajamy obrot (jeszcze nie startujemy)
  if (!turning && !turnArmed && (nowMs - lastTurnMs) >= TURN_INTERVAL_MS) {
    turnArmed = true; settleTimer = 0.0f; armMs = nowMs;
  }
  // 2) PRZYGOTOWANIE: obrot startuje dopiero, gdy robot jest pionowo i spokojny
  //     przez TURN_READY_TIME (lub po przekroczeniu TURN_ARM_TIMEOUT_MS - obroc mimo to)
  if (turnArmed) {
    bool calm = fabs(error) < TURN_READY_ANGLE
    && fabs(gyroX) < TURN_READY_RATE
    && fabs(cmdSpeed) < TURN_READY_SPEED;
    settleTimer = calm ? (settleTimer + dt) : 0.0f;
    if (settleTimer >= TURN_READY_TIME || (nowMs - armMs) >= TURN_ARM_TIMEOUT_MS) {
      turnArmed = false; turning = true; turnStepsLeft = TURN_STEPS;
    }
  }
  // --- Predkosc roznicowa obrotu z PROPORCJONALNYM priorytetem balansu ---
  float turnTarget = 0.0f;
  if (turning) {
    turnStepsLeft -= fabs(turnCmd) * dt;            // licz wykonany luk wg AKTUALNEJ predkosci
    if (turnStepsLeft <= 0.0f) { turning = false; lastTurnMs = nowMs; }
    else {
      // wspolczynnik 1 (pion) -> 0 (przechyl TURN_PITCH_STOP): obrot zwalnia, nie blokuje sie
      float m = (TURN_PITCH_STOP - fabs(error)) / (TURN_PITCH_STOP - TURN_PITCH_FULL);
      m = constrain(m, 0.0f, 1.0f);
      turnTarget = TURN_DIR * TURN_RATE * m;
    }
  }
  if (turning && fabs(error) >= TURN_PITCH_EMERG) {
    turnCmd = 0.0f;                                 // awaryjne natychmiastowe odciecie (duzy przechyl)
  } else {
    float dTurn = TURN_RAMP * dt;                   // plynna rampa
    turnCmd += constrain(turnTarget - turnCmd, -dTurn, dTurn);
  }

  // --- Opcjonalne utrzymanie kursu (yaw) - rozna predkosc kol ---
  float yawCorr = 0.0f;
  #if ENABLE_YAW_HOLD
  if (!turning && fabs(turnCmd) < 50.0f) {          // nie koliduj z zamierzonym obrotem
    heading += yawRate * dt;                        // scalony kurs
    yawCorr = -(KYAW_P * heading + KYAW_D * yawRate);
    yawCorr = constrain(yawCorr, -YAW_CORR_LIMIT, YAW_CORR_LIMIT);
  } else {
    heading = 0.0f;                                 // po obrocie trzymaj NOWY kurs
  }
  #endif

  // --- Skladanie: balans (wspolny) +/- (obrot + korekta kursu), do silnikow ---
  float rateBalance = cmdSpeed * SPEED_TO_RATE;     // mikrokroki/s
  float diff = turnCmd + yawCorr;                   // skladowa roznicowa
  setMotorRates(rateBalance - diff, rateBalance + diff);

  // --- Debug (opcjonalny) ---
  #if DEBUG_SERIAL
  if (++dbgCounter >= DEBUG_EVERY_N) {
    dbgCounter = 0;
    Serial.print("ang="); Serial.print(compAngle, 2);
    Serial.print(" tgt="); Serial.print(TARGET_ANGLE, 2); // przechwycony kat rownowagi
    Serial.print(" trim=");Serial.print(angleTrim, 2);    // auto-trim
    Serial.print(" err="); Serial.print(error, 2);
    Serial.print(" out="); Serial.print(output, 0);
    Serial.print(" st="); Serial.print(turning ? 2 : (turnArmed ? 1 : 0)); // 0=balans 1=przygot. 2=obrot
    Serial.print(" tc="); Serial.println(turnCmd, 0);   // biezaca predkosc obrotu
  }
  #endif
}
