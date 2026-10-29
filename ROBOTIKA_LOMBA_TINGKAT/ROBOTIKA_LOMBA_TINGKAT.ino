#include <Arduino.h>
#include <Preferences.h>

// CONFIG PIN

// HC-SR04 (urutan array: kiri, depan, kanan)
const uint8_t PIN_US_TRIG[3] = {25, 26, 27};   // output, bisa langsung dari ESP32 (3.3 V cukup)
const uint8_t PIN_US_ECHO[3] = {34, 35, 39};   // input-only; WAJIB lewat pembagi tegangan 5V -> 3.3V

const uint8_t PIN_AIN1 = 32;    // motor kiri
const uint8_t PIN_AIN2 = 33;
const uint8_t PIN_BIN1 = 18;    // motor kanan
const uint8_t PIN_BIN2 = 19;
const uint8_t PIN_NSLEEP = 17;  // HIGH = driver aktif

const uint8_t PIN_BTN_LEFT  = 13;
const uint8_t PIN_BTN_RIGHT = 14;
const uint8_t PIN_LED_RED   = 4;
const uint8_t PIN_LED_GREEN = 16;

// 2. CONST HARDWARE

const uint32_t PWM_FREQ = 20000;   // 20 kHz: senyap, riak arus kecil, DRV8833 mendukung
const uint8_t  PWM_RES  = 8;       // 0..255
const int      PWM_MAX  = 255;

// Ubah true jika arah putar motor terbalik karena pengkabelan
const bool MOTOR_L_INVERT = false;
const bool MOTOR_R_INVERT = false;
// Kompensasi motor tidak identik (1.00 = tanpa koreksi)
const float MOTOR_L_GAIN = 1.00f;
const float MOTOR_R_GAIN = 1.00f;

const int SLEW_UP   = 22;   // batas kenaikan PWM per loop (anti selip saat start)
const int SLEW_DOWN = 80;   // batas penurunan PWM per loop

// Sensor ultrasonik HC-SR04
const uint16_t US_MAX_RANGE_MM  = 1200;   // mm, nilai di atas ini / tanpa echo dianggap "jauh"
const uint16_t US_MIN_RANGE_MM  = 20;     // batas bawah fisik HC-SR04 (~2 cm)
const uint32_t US_TIMEOUT_US    = 7500;   // batas tunggu echo (~1,28 m pulang-pergi)
const uint8_t  US_TRIG_PULSE_US = 10;     // lebar pulsa trigger sesuai datasheet
const float    SIDE_ALPHA       = 0.6f;   // bobot EMA sensor samping (makin kecil makin halus)
const int      STUCK_DELTA_MM   = 15;     // toleransi derau untuk deteksi macet (HC-SR04 lebih berisik)
// true  = robot ERROR bila ada sensor yang tidak memberi echo sama sekali saat boot
// false = lewati self-test (berguna saat uji di meja tanpa dinding)
const bool REQUIRE_SENSOR_SELFTEST = true;

const uint32_t DEBOUNCE_MS      = 30;
const uint32_t SELECT_WINDOW_MS = 1200;
const uint32_t START_DELAY_MS   = 400;
const uint32_t FINISH_GRACE_MS  = 1500;  // deteksi finish diabaikan di awal
const uint32_t OPEN_GRACE_MS    = 250;   // abaikan "dinding hilang" setelah belokan

// 3. PARAMETER TUNING (disimpan ke flash dengan perintah "save")
const uint16_t PARAMS_VER = 4;   // dinaikkan karena struktur parameter berubah

struct Params {
  uint16_t ver = PARAMS_VER;
  // PID
  float kp = 0.60f;          // PWM per mm error
  float ki = 0.00f;          // biasanya 0
  float kd = 0.02f;          // PWM per (mm/detik), diturunkan karena derau HC-SR04 lebih besar
  int   maxCorr = 60;        // batas koreksi PID (PWM)
  // Kecepatan (PWM 0..255)
  int baseSpeed = 95;
  int maxSpeed  = 115;       // batas keras, lindungi motor Sprint Dash
  int turnSpeed = 85;
  int minSpeed  = 45;
  // Jarak (mm, dari muka sensor)
  int targetSide = 110;      // jarak ideal ke dinding yang diikuti
  int openSide   = 250;      // di atas ini dinding dianggap hilang
  int frontTurn  = 120;      // di bawah ini wajib belok
  int frontSlow  = 260;      // di bawah ini mulai melambat
  int frontClear = 260;      // pivot selesai jika depan sudah lega
  // Sensor
  uint16_t usGapUs = 2500;   // jeda antar pembacaan sensor (us), cegah crosstalk/gema
  // Manuver
  int      arcPct         = 35;    // kecepatan roda dalam saat arc (% turnSpeed)
  uint16_t outerAdvanceMs = 120;   // maju sebelum arc di tikungan terbuka
  uint16_t recoverMs      = 300;   // durasi mundur saat recovery
  uint16_t turnTimeout    = 2200;  // batas waktu satu manuver
  uint16_t stuckMs        = 2500;  // dianggap macet jika tidak ada perubahan
  int      stuckFrontMax  = 500;   // cek macet hanya jika depan lebih dekat dari ini
  // Finish & batas waktu
  bool     finishDetect = true;
  int      finishSide   = 700;     // kiri DAN kanan di atas ini = keluar arena
  uint16_t finishHold   = 350;     // harus bertahan selama ini (ms)
  uint32_t runTimeout   = 90000;   // berhenti otomatis (ms)
  // Debug
  bool telemetry = false;
};

Params P;
Preferences prefs;

// 4. KONFIGURASI CHECKPOINT (START + 3 titik RETRY)

// followOverride : -1 = ikut aturan track, 0 = paksa dinding kiri, 1 = paksa dinding kanan
// startTurn      : -1 = pivot kiri, 0 = tidak, 1 = pivot kanan (sebelum mulai)
// startTurnMs    : lama pivot awal
// speedPct       : skala kecepatan (%) untuk titik ini
// Robot tanpa memori, jadi dari titik mana pun cukup meraba dinding lalu mengikutinya.
// Kolom ini disediakan untuk penyesuaian setelah uji di arena sebenarnya.

struct CheckpointCfg {
  const char* name;
  int8_t   followOverride;
  int8_t   startTurn;
  uint16_t startTurnMs;
  uint8_t  speedPct;
};

const CheckpointCfg CP_CFG[4] = {
  {"START",   -1, 0, 0, 100},
  {"RETRY-1", -1, 0, 0,  90},
  {"RETRY-2", -1, 0, 0,  90},
  {"RETRY-3", -1, 0, 0,  90}
};

// Track KIRI mengikuti dinding KANAN (dinding tengah) dan sebaliknya.
// Verifikasi dengan arena tiruan, ubah ke false jika ternyata sebaliknya.
const bool LEFT_TRACK_FOLLOWS_RIGHT_WALL = true;

// 5. TIPE DATA & STATE
enum SensorIdx : uint8_t { SENS_L = 0, SENS_F = 1, SENS_R = 2 };

enum SysState : uint8_t {
  SYS_IDLE, SYS_READY, SYS_RUN, SYS_FINISHED, SYS_ERROR, SYS_MAINT
};
const char* SYS_NAMES[] = {"IDLE", "READY", "RUN", "FINISHED", "ERROR", "MAINT"};

enum NavState : uint8_t {
  NAV_START_TURN, NAV_ACQUIRE, NAV_FOLLOW, NAV_TURN_OUTER,
  NAV_PIVOT, NAV_RECOVER
};
const char* NAV_NAMES[] = {"START_TURN", "ACQUIRE", "FOLLOW", "TURN_OUTER",
                           "PIVOT", "RECOVER"};

enum LedMode : uint8_t { LED_OFF, LED_ON, LED_BLINK_SLOW, LED_BLINK_SLOW_INV, LED_BLINK_FAST };

SysState sys = SYS_IDLE;
NavState nav = NAV_FOLLOW;

bool     sensorsReady = false;
uint16_t dist[3] = {US_MAX_RANGE_MM, US_MAX_RANGE_MM, US_MAX_RANGE_MM};
// Riwayat 3 pembacaan mentah per sensor untuk median filter (membuang lonjakan palsu HC-SR04)
uint16_t rawHist[3][3] = {
  {US_MAX_RANGE_MM, US_MAX_RANGE_MM, US_MAX_RANGE_MM},
  {US_MAX_RANGE_MM, US_MAX_RANGE_MM, US_MAX_RANGE_MM},
  {US_MAX_RANGE_MM, US_MAX_RANGE_MM, US_MAX_RANGE_MM}
};
uint8_t histIdx[3] = {0, 0, 0};

// Pilihan operator
bool    trackRight = false;   // false = track kiri, true = track kanan
uint8_t selCp = 0;            // 0..3
bool    followRight = true;   // dinding yang diikuti
uint8_t cpSpeedPct = 100;

// Navigasi
uint32_t navT0 = 0, runStartMs = 0, finishTimer = 0, graceUntil = 0;
int      pivotDir = 1;
uint8_t  openCount = 0;
int      lastErr = 0, lastCorr = 0;
int      lastSig = 0;
uint32_t lastSigT = 0;

// PID
float    pidI = 0, pidPrevErr = 0;
uint32_t pidPrevT = 0;
bool     pidFirst = true;

// Motor
int curL = 0, curR = 0, cmdL = 0, cmdR = 0;

// LED
LedMode redMode = LED_OFF, greenMode = LED_OFF;

// Tombol
struct Button {
  uint8_t  pin;
  bool     down = false, rawLast = false, evPress = false, evRelease = false;
  uint32_t tChange = 0, tDown = 0, lastHold = 0;
  explicit Button(uint8_t p) : pin(p) {}
};
Button btnL(PIN_BTN_LEFT), btnR(PIN_BTN_RIGHT);

// Pemilihan
uint8_t  pressCount = 0;
uint32_t lastPressT = 0;
bool     selRight = false;

// Maintenance
bool streamSensors = false;

// 6. LED
void ledApply(uint8_t pin, LedMode m) {
  uint32_t t = millis();
  bool v = false;
  switch (m) {
    case LED_ON:             v = true; break;
    case LED_BLINK_SLOW:     v = ((t / 500) % 2) == 0; break;
    case LED_BLINK_SLOW_INV: v = ((t / 500) % 2) != 0; break;
    case LED_BLINK_FAST:     v = ((t / 120) % 2) == 0; break;
    default:                 v = false;
  }
  digitalWrite(pin, v ? HIGH : LOW);
}

void ledsSet(LedMode r, LedMode g) { redMode = r; greenMode = g; }
void ledsUpdate() { ledApply(PIN_LED_RED, redMode); ledApply(PIN_LED_GREEN, greenMode); }

// Berkedip n kali (blocking, hanya dipakai saat konfirmasi pilihan)
void blinkN(uint8_t pin, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    digitalWrite(pin, HIGH); delay(180);
    digitalWrite(pin, LOW);  delay(180);
  }
  delay(250);
}

// 7. TOMBOL
void btnUpdate(Button &b) {
  b.evPress = b.evRelease = false;
  bool raw = (digitalRead(b.pin) == LOW);   // aktif LOW (INPUT_PULLUP)
  uint32_t now = millis();
  if (raw != b.rawLast) { b.rawLast = raw; b.tChange = now; }
  if ((now - b.tChange) >= DEBOUNCE_MS && raw != b.down) {
    b.down = raw;
    if (raw) { b.evPress = true; b.tDown = now; }
    else     { b.evRelease = true; b.lastHold = now - b.tDown; }
  }
}

// 8. MOTOR DRIVER DRV8833 (PWM di pin IN, slow decay)

// Maju  : IN1 = HIGH penuh, IN2 = PWM terbalik
// Mundur: IN2 = HIGH penuh, IN1 = PWM terbalik
// Speed 0 = rem (kedua pin HIGH)
void motorWrite(uint8_t pinA, uint8_t pinB, int speed) {
  speed = constrain(speed, -P.maxSpeed, P.maxSpeed);
  if (speed > 0) {
    ledcWrite(pinA, PWM_MAX);
    ledcWrite(pinB, PWM_MAX - speed);
  } else if (speed < 0) {
    ledcWrite(pinB, PWM_MAX);
    ledcWrite(pinA, PWM_MAX + speed);
  } else {
    ledcWrite(pinA, PWM_MAX);
    ledcWrite(pinB, PWM_MAX);
  }
}

void motorsInit() {
  pinMode(PIN_NSLEEP, OUTPUT);
  digitalWrite(PIN_NSLEEP, LOW);            // driver mati saat boot
  ledcAttach(PIN_AIN1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_AIN2, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_BIN1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_BIN2, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_AIN1, 0); ledcWrite(PIN_AIN2, 0);
  ledcWrite(PIN_BIN1, 0); ledcWrite(PIN_BIN2, 0);
}

int slew(int cur, int tgt) {
  int d = tgt - cur;
  d = constrain(d, -SLEW_DOWN, SLEW_UP);
  return cur + d;
}

// Perintah kecepatan kiri/kanan (-255..255)
void setMotors(int l, int r) {
  digitalWrite(PIN_NSLEEP, HIGH);
  cmdL = l; cmdR = r;
  curL = slew(curL, l);
  curR = slew(curR, r);
  int outL = (int)(curL * MOTOR_L_GAIN) * (MOTOR_L_INVERT ? -1 : 1);
  int outR = (int)(curR * MOTOR_R_GAIN) * (MOTOR_R_INVERT ? -1 : 1);
  motorWrite(PIN_AIN1, PIN_AIN2, outL);
  motorWrite(PIN_BIN1, PIN_BIN2, outR);
}

void motorsBrake() {
  curL = curR = cmdL = cmdR = 0;
  motorWrite(PIN_AIN1, PIN_AIN2, 0);
  motorWrite(PIN_BIN1, PIN_BIN2, 0);
}

void motorsOff() {
  motorsBrake();
  ledcWrite(PIN_AIN1, 0); ledcWrite(PIN_AIN2, 0);
  ledcWrite(PIN_BIN1, 0); ledcWrite(PIN_BIN2, 0);
  digitalWrite(PIN_NSLEEP, LOW);
}

// dir = +1 putar kanan (searah jarum jam), -1 putar kiri, di tempat
void pivotMotors(int dir, int spd) { setMotors(dir * spd, -dir * spd); }

// 9. SENSOR HC-SR04 (3 unit, dibaca bergantian)

// Satu pengukuran. Return false jika tidak ada echo dalam batas waktu (anggap "jauh").
bool usMeasure(uint8_t i, uint16_t &mm) {
  digitalWrite(PIN_US_TRIG[i], LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_US_TRIG[i], HIGH);
  delayMicroseconds(US_TRIG_PULSE_US);
  digitalWrite(PIN_US_TRIG[i], LOW);

  uint32_t d = pulseIn(PIN_US_ECHO[i], HIGH, US_TIMEOUT_US);   // 0 = timeout
  if (d == 0) return false;

  uint32_t v = d * 343UL / 2000UL;   // mm = us * 0.343 mm/us / 2 (pulang-pergi)
  if (v > US_MAX_RANGE_MM) v = US_MAX_RANGE_MM;
  if (v < US_MIN_RANGE_MM) v = US_MIN_RANGE_MM;
  mm = (uint16_t)v;
  return true;
}

uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  uint16_t lo = (a < b) ? a : b;
  uint16_t hi = (a < b) ? b : a;
  return (c < lo) ? lo : ((c > hi) ? hi : c);
}

bool sensorsInit() {
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(PIN_US_TRIG[i], OUTPUT);
    digitalWrite(PIN_US_TRIG[i], LOW);
    pinMode(PIN_US_ECHO[i], INPUT);
  }
  delay(50);

  // Self-test: tiap sensor harus memberi minimal satu echo valid dalam 5 percobaan
  bool ok = true;
  for (uint8_t i = 0; i < 3; i++) {
    if (digitalRead(PIN_US_ECHO[i]) == HIGH)
      Serial.printf("[US] Sensor %u: pin ECHO tertahan HIGH (cek kabel/pembagi tegangan)\n", i);
    bool alive = false;
    uint16_t mm = 0;
    for (uint8_t t = 0; t < 5 && !alive; t++) {
      alive = usMeasure(i, mm);
      delay(30);
    }
    if (alive) Serial.printf("[US] Sensor %u OK (%u mm)\n", i, mm);
    else {
      Serial.printf("[US] Sensor %u TIDAK ADA ECHO (cek kabel/VCC 5V, atau tidak ada dinding < 1,2 m)\n", i);
      ok = false;
    }
  }
  if (!ok && !REQUIRE_SENSOR_SELFTEST) {
    Serial.println("[US] Self-test dilewati (REQUIRE_SENSOR_SELFTEST=false)");
    return true;
  }
  return ok;
}

// Baca 3 sensor bergantian. Hasil: median 3 sampel, sensor samping ditambah EMA.
void readSensors() {
  for (uint8_t i = 0; i < 3; i++) {
    uint16_t mm;
    if (!usMeasure(i, mm)) mm = US_MAX_RANGE_MM;      // tanpa echo = jauh

    rawHist[i][histIdx[i]] = mm;
    histIdx[i] = (histIdx[i] + 1) % 3;
    uint16_t m = median3(rawHist[i][0], rawHist[i][1], rawHist[i][2]);

    if (i == SENS_F) dist[i] = m;                                              // depan: median saja (cepat)
    else             dist[i] = (uint16_t)(SIDE_ALPHA * m + (1.0f - SIDE_ALPHA) * dist[i]);  // samping: EMA

    delayMicroseconds(P.usGapUs);    // beri waktu gema mereda sebelum sensor berikutnya
  }
}

// Isi ulang riwayat sebelum mulai jalan agar tidak memakai data lama
void sensorsPrime() {
  for (uint8_t k = 0; k < 3; k++) readSensors();
  for (uint8_t i = 0; i < 3; i++) dist[i] = median3(rawHist[i][0], rawHist[i][1], rawHist[i][2]);
}

inline int followDist() { return followRight ? dist[SENS_R] : dist[SENS_L]; }

// 10. PID
void pidReset() { pidI = 0; pidPrevErr = 0; pidFirst = true; }

int pidCompute(int err) {
  uint32_t now = millis();
  float dt = (now - pidPrevT) / 1000.0f;
  if (pidFirst || dt <= 0.0f || dt > 0.2f) { dt = 0.02f; pidPrevErr = err; pidFirst = false; }
  pidI += err * dt;
  float lim = (P.ki > 0.0f) ? (P.maxCorr / P.ki) : 0.0f;   // anti-windup
  pidI = constrain(pidI, -lim, lim);
  float d = (err - pidPrevErr) / dt;
  pidPrevErr = err;
  pidPrevT = now;
  float out = P.kp * err + P.ki * pidI + P.kd * d;
  return constrain((int)out, -P.maxCorr, P.maxCorr);
}

// 11. NAVIGASI (state machine)
void navEnter(NavState s) {
  nav = s;
  navT0 = millis();
  lastSig = dist[0] + dist[1] + dist[2];
  lastSigT = navT0;
  if (s == NAV_FOLLOW) { pidReset(); openCount = 0; graceUntil = navT0 + OPEN_GRACE_MS; }
  if (P.telemetry) Serial.printf("[NAV] -> %s\n", NAV_NAMES[s]);
}

NavState decideAfterStart() {
  return (followDist() > P.openSide && dist[SENS_F] > P.frontTurn) ? NAV_ACQUIRE : NAV_FOLLOW;
}

// Macet jika semua pembacaan nyaris tidak berubah padahal depan dekat
bool isStuck(uint32_t now) {
  int sum = dist[0] + dist[1] + dist[2];
  if (abs(sum - lastSig) > STUCK_DELTA_MM || dist[SENS_F] > P.stuckFrontMax) { lastSig = sum; lastSigT = now; }
  return (now - lastSigT) > P.stuckMs;
}

void navStep() {
  const uint32_t now = millis();
  const int F = dist[SENS_F];
  const int S = followDist();
  const int dir = followRight ? 1 : -1;        // +1: dinding di kanan
  const int towardDir = dir;                    // pivot ke arah dinding
  const int awayDir = -dir;                     // pivot menjauhi dinding
  const int baseS = P.baseSpeed * cpSpeedPct / 100;

  switch (nav) {

    // Pivot awal paksa (opsional per checkpoint)
    case NAV_START_TURN: {
      const CheckpointCfg &c = CP_CFG[selCp];
      if (now - navT0 >= c.startTurnMs) navEnter(decideAfterStart());
      else pivotMotors(c.startTurn, P.turnSpeed);
      break;
    }

    // Belum ada dinding di sisi yang diikuti: maju sambil sedikit mengarah ke sana
    case NAV_ACQUIRE: {
      setMotors(P.turnSpeed + dir * 10, P.turnSpeed - dir * 10);
      if (F < P.frontTurn) { pivotDir = (S > P.openSide) ? towardDir : awayDir; navEnter(NAV_PIVOT); }
      else if (S < P.openSide) navEnter(NAV_FOLLOW);
      else if (now - navT0 > 3000) navEnter(NAV_FOLLOW);
      else if (isStuck(now)) navEnter(NAV_RECOVER);
      break;
    }

    // Ikuti dinding dengan PID
    case NAV_FOLLOW: {
      if (F < P.frontTurn) {
        // Depan tertutup: jika sisi ikut terbuka belok ke sana, selain itu menjauh (juga U-turn)
        pivotDir = (S > P.openSide) ? towardDir : awayDir;
        navEnter(NAV_PIVOT);
        break;
      }
      if (now > graceUntil && S > P.openSide) {
        if (++openCount >= 2) { navEnter(NAV_TURN_OUTER); break; }
      } else {
        openCount = 0;
      }
      if (isStuck(now)) { navEnter(NAV_RECOVER); break; }

      int err = constrain(S - P.targetSide, -150, 150);
      int corr = pidCompute(err);
      lastErr = err; lastCorr = corr;

      // Profil kecepatan: melambat mendekati dinding depan
      int s = baseS;
      if (F < P.frontSlow) {
        int sLow = min(P.minSpeed, baseS);
        s = map(F, P.frontTurn, P.frontSlow, sLow, baseS);
        s = constrain(s, sLow, baseS);
      }
      // Error positif = terlalu jauh dari dinding, arahkan ke dinding
      int l = constrain(s + dir * corr, 0, P.maxSpeed);
      int r = constrain(s - dir * corr, 0, P.maxSpeed);
      setMotors(l, r);
      break;
    }

    // Dinding hilang (tikungan luar): maju sedikit lalu arc ke arah dinding
    case NAV_TURN_OUTER: {
      uint32_t t = now - navT0;
      if (t < P.outerAdvanceMs) {
        setMotors(P.turnSpeed, P.turnSpeed);
      } else {
        int inner = P.turnSpeed * P.arcPct / 100;
        if (followRight) setMotors(P.turnSpeed, inner);
        else             setMotors(inner, P.turnSpeed);
        if (F < P.frontTurn) navEnter(NAV_FOLLOW);
        else if (t > (uint32_t)P.outerAdvanceMs + 120 && S < P.openSide) navEnter(NAV_FOLLOW);
        else if (t > P.turnTimeout) navEnter(NAV_ACQUIRE);
      }
      if (isStuck(now)) navEnter(NAV_RECOVER);
      break;
    }

    // Putar di tempat sampai depan lega
    case NAV_PIVOT: {
      pivotMotors(pivotDir, P.turnSpeed);
      uint32_t t = now - navT0;
      if (t > 150 && F >= P.frontClear) navEnter(NAV_FOLLOW);
      else if (t > P.turnTimeout) navEnter(NAV_RECOVER);
      break;
    }

    // Mundur sebentar lalu pivot menjauhi dinding
    case NAV_RECOVER: {
      setMotors(-P.turnSpeed, -P.turnSpeed);
      if (now - navT0 > P.recoverMs) { pivotDir = awayDir; navEnter(NAV_PIVOT); }
      break;
    }
  }
}

// 12. STATE SISTEM
void setSys(SysState s) {
  sys = s;
  Serial.printf("[SYS] %s\n", SYS_NAMES[s]);
  switch (s) {
    case SYS_IDLE:     motorsOff(); ledsSet(LED_BLINK_SLOW, LED_OFF); break;
    case SYS_READY:    motorsOff(); ledsSet(LED_OFF, LED_ON); break;
    case SYS_RUN:      ledsSet(LED_OFF, LED_BLINK_FAST); break;
    case SYS_FINISHED: motorsBrake(); ledsSet(LED_ON, LED_ON); break;
    case SYS_ERROR:    motorsOff(); ledsSet(LED_BLINK_FAST, LED_OFF); break;
    case SYS_MAINT:    motorsOff(); ledsSet(LED_BLINK_SLOW, LED_BLINK_SLOW_INV); break;
  }
}

// Terapkan pilihan track + checkpoint menjadi aturan dinding & kecepatan
void applySelection() {
  const CheckpointCfg &c = CP_CFG[selCp];
  if (c.followOverride >= 0) followRight = (c.followOverride == 1);
  else followRight = trackRight ? !LEFT_TRACK_FOLLOWS_RIGHT_WALL : LEFT_TRACK_FOLLOWS_RIGHT_WALL;
  cpSpeedPct = c.speedPct;
}

void startRun() {
  if (!sensorsReady) { setSys(SYS_ERROR); return; }
  applySelection();
  Serial.printf("[RUN] track=%s titik=%s ikuti dinding=%s kecepatan=%u%%\n",
                trackRight ? "KANAN" : "KIRI", CP_CFG[selCp].name,
                followRight ? "KANAN" : "KIRI", cpSpeedPct);
  delay(START_DELAY_MS);
  sensorsPrime();
  curL = curR = 0;
  runStartMs = millis();
  finishTimer = 0;
  pidReset();
  const CheckpointCfg &c = CP_CFG[selCp];
  if (c.startTurn != 0 && c.startTurnMs > 0) navEnter(NAV_START_TURN);
  else navEnter(decideAfterStart());
  setSys(SYS_RUN);
}

void endRun(const char* reason, bool success) {
  Serial.printf("[RUN] selesai: %s\n", reason);
  motorsBrake();
  setSys(success ? SYS_FINISHED : SYS_IDLE);
}

void runStep() {
  readSensors();
  uint32_t now = millis();

  if (btnL.evPress || btnR.evPress) { endRun("berhenti manual", false); return; }
  if (now - runStartMs > P.runTimeout) { endRun("timeout", false); return; }

  // Deteksi finish: kiri DAN kanan sama-sama terbuka (keluar lorong) cukup lama
  if (P.finishDetect && nav != NAV_ACQUIRE && (now - runStartMs) > FINISH_GRACE_MS) {
    if (dist[SENS_L] >= P.finishSide && dist[SENS_R] >= P.finishSide && dist[SENS_F] > P.frontClear) {
      if (finishTimer == 0) finishTimer = now;
      else if (now - finishTimer >= P.finishHold) { endRun("finish terdeteksi", true); return; }
    } else {
      finishTimer = 0;
    }
  }

  navStep();

  if (P.telemetry) {
    static uint32_t lastPrint = 0;
    if (now - lastPrint >= 100) {
      lastPrint = now;
      Serial.printf("T,%lu,%s,L%d,F%d,R%d,err%d,corr%d,m%d/%d\n", (unsigned long)(now - runStartMs),
                    NAV_NAMES[nav], dist[SENS_L], dist[SENS_F], dist[SENS_R], lastErr, lastCorr, cmdL, cmdR);
    }
  }
}

// IDLE: pilih track dengan jumlah tekan = titik mulai
void idleStep() {
  uint32_t now = millis();
  if ((btnL.evPress || btnR.evPress) && !(btnL.down && btnR.down)) {
    bool right = btnR.evPress;
    if (pressCount == 0) { selRight = right; pressCount = 1; }
    else if (right == selRight && pressCount < 4) pressCount++;
    lastPressT = now;
  }
  if (pressCount > 0 && (now - lastPressT) > SELECT_WINDOW_MS) {
    trackRight = selRight;
    selCp = pressCount - 1;
    pressCount = 0;
    applySelection();
    Serial.printf("[SEL] track=%s titik=%s ikuti dinding=%s\n", trackRight ? "KANAN" : "KIRI",
                  CP_CFG[selCp].name, followRight ? "KANAN" : "KIRI");
    digitalWrite(PIN_LED_RED, LOW); digitalWrite(PIN_LED_GREEN, LOW);
    blinkN(PIN_LED_RED, trackRight ? 2 : 1);
    blinkN(PIN_LED_GREEN, selCp + 1);
    setSys(SYS_READY);
  }
}

// READY: tekan sebentar = jalan, tahan >1 detik = kembali pilih
void readyStep() {
  if (btnL.evRelease || btnR.evRelease) {
    uint32_t hold = btnL.evRelease ? btnL.lastHold : btnR.lastHold;
    if (hold >= 1000) setSys(SYS_IDLE);
    else startRun();
  }
}

// FINISHED / ERROR: tombol untuk lanjut
void finishedStep() { if (btnL.evRelease || btnR.evRelease) setSys(SYS_IDLE); }
void errorStep()    { if (btnL.evRelease || btnR.evRelease) ESP.restart(); }

// MAINT: KIRI = tes motor, KANAN = keluar
void motorTest() {
  Serial.println("[MAINT] Tes motor: kiri maju/mundur, kanan maju/mundur");
  int s = P.turnSpeed;
  setMotors(s, 0);  delay(800); setMotors(0, 0); delay(300);
  setMotors(-s, 0); delay(800); setMotors(0, 0); delay(300);
  setMotors(0, s);  delay(800); setMotors(0, 0); delay(300);
  setMotors(0, -s); delay(800); motorsOff();
  Serial.println("[MAINT] Tes motor selesai");
}

void maintStep() {
  if (btnL.evPress) motorTest();
  if (btnR.evPress) { setSys(SYS_IDLE); return; }
  static uint32_t lastPrint = 0;
  if (streamSensors && millis() - lastPrint >= 200) {
    lastPrint = millis();
    readSensors();
    Serial.printf("L=%u F=%u R=%u mm\n", dist[SENS_L], dist[SENS_F], dist[SENS_R]);
  }
}

// =====================================================================
// 13. PENYIMPANAN & SERIAL CLI (maintenance & tuning cepat)
// =====================================================================
void paramsLoad() {
  Params tmp;
  size_t n = prefs.getBytes("cfg", &tmp, sizeof(tmp));
  if (n == sizeof(Params) && tmp.ver == PARAMS_VER) { P = tmp; Serial.println("[CFG] dimuat dari flash"); }
  else Serial.println("[CFG] memakai nilai default");
}
void paramsSave() { prefs.putBytes("cfg", &P, sizeof(P)); Serial.println("[CFG] tersimpan"); }

void printConfig() {
  Serial.printf("kp=%.3f ki=%.3f kd=%.3f mc=%d\n", P.kp, P.ki, P.kd, P.maxCorr);
  Serial.printf("bs=%d ms=%d ts=%d mn=%d\n", P.baseSpeed, P.maxSpeed, P.turnSpeed, P.minSpeed);
  Serial.printf("tg=%d op=%d ft=%d fs=%d fc=%d sg=%u\n", P.targetSide, P.openSide, P.frontTurn, P.frontSlow, P.frontClear, P.usGapUs);
  Serial.printf("arc=%d adv=%u rec=%u tto=%u stk=%u\n", P.arcPct, P.outerAdvanceMs, P.recoverMs, P.turnTimeout, P.stuckMs);
  Serial.printf("fin=%d fside=%d fhold=%u rto=%lu tel=%d\n", P.finishDetect, P.finishSide, P.finishHold,
                (unsigned long)P.runTimeout, P.telemetry);
  Serial.printf("side=%s cp=%d state=%s sensor=%s\n", trackRight ? "r" : "l", selCp, SYS_NAMES[sys],
                sensorsReady ? "OK" : "GAGAL");
}

void printHelp() {
  Serial.println(F("== PERINTAH =="));
  Serial.println(F("show | save | load | reset | state | sn (alir sensor) | mt (tes motor)"));
  Serial.println(F("go | stop | side=l/r | cp=0..3"));
  Serial.println(F("PID: kp= ki= kd= mc=   Kecepatan: bs= ms= ts= mn="));
  Serial.println(F("Jarak(mm): tg= op= ft= fs= fc=   Sensor: sg= (jeda us)   Manuver: arc= adv= rec= tto= stk="));
  Serial.println(F("Finish: fin=0/1 fside= fhold= rto=   Debug: tel=0/1"));
}

void handleCommand(String line) {
  line.trim(); line.toLowerCase();
  int eq = line.indexOf('=');
  String key = (eq < 0) ? line : line.substring(0, eq);
  String val = (eq < 0) ? "" : line.substring(eq + 1);
  float v = val.toFloat();

  if      (key == "help")  printHelp();
  else if (key == "show")  printConfig();
  else if (key == "save")  paramsSave();
  else if (key == "load")  paramsLoad();
  else if (key == "reset") { P = Params(); Serial.println("[CFG] default dipulihkan (belum disimpan)"); }
  else if (key == "state") Serial.printf("sys=%s nav=%s\n", SYS_NAMES[sys], NAV_NAMES[nav]);
  else if (key == "sn")    { streamSensors = !streamSensors; Serial.printf("stream sensor %s\n", streamSensors ? "ON" : "OFF"); }
  else if (key == "mt")    { if (sys != SYS_RUN) motorTest(); }
  else if (key == "go")    { if (sys == SYS_IDLE || sys == SYS_READY) startRun(); }
  else if (key == "stop")  { if (sys == SYS_RUN) endRun("perintah serial", false); else motorsOff(); }
  else if (key == "side")  { trackRight = val.startsWith("r"); applySelection(); }
  else if (key == "cp")    { selCp = constrain((int)v, 0, 3); applySelection(); }
  else if (key == "kp")    P.kp = v;
  else if (key == "ki")    P.ki = v;
  else if (key == "kd")    P.kd = v;
  else if (key == "mc")    P.maxCorr = (int)v;
  else if (key == "bs")    P.baseSpeed = constrain((int)v, 0, 255);
  else if (key == "ms")    P.maxSpeed = constrain((int)v, 0, 255);
  else if (key == "ts")    P.turnSpeed = constrain((int)v, 0, 255);
  else if (key == "mn")    P.minSpeed = constrain((int)v, 0, 255);
  else if (key == "tg")    P.targetSide = (int)v;
  else if (key == "op")    P.openSide = (int)v;
  else if (key == "ft")    P.frontTurn = (int)v;
  else if (key == "fs")    P.frontSlow = (int)v;
  else if (key == "fc")    P.frontClear = (int)v;
  else if (key == "sg")    P.usGapUs = (uint16_t)constrain((int)v, 0, 20000);
  else if (key == "arc")   P.arcPct = constrain((int)v, 0, 100);
  else if (key == "adv")   P.outerAdvanceMs = (uint16_t)v;
  else if (key == "rec")   P.recoverMs = (uint16_t)v;
  else if (key == "tto")   P.turnTimeout = (uint16_t)v;
  else if (key == "stk")   P.stuckMs = (uint16_t)v;
  else if (key == "fin")   P.finishDetect = (v != 0);
  else if (key == "fside") P.finishSide = (int)v;
  else if (key == "fhold") P.finishHold = (uint16_t)v;
  else if (key == "rto")   P.runTimeout = (uint32_t)v;
  else if (key == "tel")   P.telemetry = (v != 0);
  else { Serial.println("Perintah tidak dikenal. Ketik: help"); return; }
  Serial.println("OK");
}

void cliPoll() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (buf.length()) { handleCommand(buf); buf = ""; } }
    else if (buf.length() < 48) buf += c;
  }
}

// 14. SETUP & LOOP
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Wall Follower ESP32 (HC-SR04) ===");

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);

  digitalWrite(PIN_LED_RED, HIGH); digitalWrite(PIN_LED_GREEN, HIGH);
  motorsInit();

  prefs.begin("wallf", false);
  paramsLoad();

  sensorsReady = sensorsInit();

  delay(400);   // LED boot menyala sebentar
  digitalWrite(PIN_LED_RED, LOW); digitalWrite(PIN_LED_GREEN, LOW);

  bool bothHeld = (digitalRead(PIN_BTN_LEFT) == LOW) && (digitalRead(PIN_BTN_RIGHT) == LOW);

  if (!sensorsReady)      { setSys(SYS_ERROR); }
  else if (bothHeld)      { streamSensors = true; setSys(SYS_MAINT); }
  else                    { setSys(SYS_IDLE); }

  printHelp();
}

void loop() {
  btnUpdate(btnL);
  btnUpdate(btnR);
  cliPoll();

  switch (sys) {
    case SYS_IDLE:     idleStep();     break;
    case SYS_READY:    readyStep();    break;
    case SYS_RUN:      runStep();      break;
    case SYS_FINISHED: finishedStep(); break;
    case SYS_ERROR:    errorStep();    break;
    case SYS_MAINT:    maintStep();    break;
  }
  ledsUpdate();
}