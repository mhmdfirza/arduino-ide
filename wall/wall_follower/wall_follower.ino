/*
 * =====================================================================
 *  WALL FOLLOWER ROBOT - ESP32 DevKit V1 (WROOM-32) + 3x HC-SR04 + DRV8833
 *  Core: ESP32 Arduino 3.x (ledcAttach / ledcWrite)   Library tambahan: tidak ada
 * =====================================================================
 *  PEMETAAN PIN
 *   HC-SR04 KIRI  : TRIG 25  ECHO 34      HC-SR04 DEPAN : TRIG 26  ECHO 35
 *   HC-SR04 KANAN : TRIG 27  ECHO 13      (semua ECHO lewat pembagi 1k + 2k)
 *   DRV8833       : IN1 19, IN2 18 (motor A / kiri)   IN3 33, IN4 32 (motor B / kanan)
 *                   EEP (nSLEEP) 17
 *   Tombol        : KIRI 23, KANAN 22 (ke GND, pull-up internal)
 *   LED           : MERAH 4, HIJAU 21 (resistor seri 220 ohm)
 *
 *  CARA PAKAI
 *   1. Nyalakan robot: LED merah berkedip pelan = IDLE (siap memilih).
 *   2. Pilih track + titik mulai:
 *        tombol KIRI = track kiri, tombol KANAN = track kanan
 *        jumlah tekan: 1x START, 2x RETRY-1, 3x RETRY-2, 4x RETRY-3
 *      LED merah menyala tetap selama menghitung. Menekan tombol sisi lain
 *      di tengah pilihan = hitung ulang dari awal.
 *   3. Konfirmasi otomatis: merah berkedip 1x (kiri) / 2x (kanan), lalu hijau
 *      berkedip n kali (n = nomor titik). Hijau menyala tetap = READY.
 *   4. Tekan tombol SISI YANG SAMA dengan track terpilih (sebentar) = JALAN.
 *      Tombol sisi lain diabaikan (merah 3x). Tahan > 1 detik = pilih ulang.
 *   5. Saat jalan, tekan tombol apa saja = berhenti darurat.
 *   6. Tahan KEDUA tombol saat menyalakan robot = MODE MAINTENANCE.
 *
 *  STATUS LED
 *   IDLE        : merah kedip pelan
 *   Memilih     : merah menyala tetap
 *   READY       : hijau menyala tetap
 *   RUN         : hijau kedip cepat (merah ikut menyala saat RECOVER / macet)
 *   FINISHED    : merah + hijau menyala
 *   Gagal/stop  : merah kedip 2x lalu kembali IDLE
 *   Start ditolak: merah kedip 3x (sensor tidak valid), robot tetap READY
 *   ERROR       : merah kedip cepat (sensor gagal saat boot), tekan tombol = restart
 *   MAINTENANCE : merah / hijau kedip bergantian
 *
 *  Serial 115200: ketik "help" untuk daftar perintah tuning.
 * =====================================================================
 */

 /**
 * ============================================================================
 *                    PANDUAN PERINTAH DAN PARAMETER SERIAL
 * ============================================================================
 * 
 * ----------------------------------------------------------------------------
 * 1. PERINTAH SERIAL MONITOR (Command List)
 * ----------------------------------------------------------------------------
 *  Perintah          Arti / Fungsi
 *  ---------------------------------------------------------------------------
 *  help            : Tampilkan daftar perintah
 *  show            : Tampilkan semua parameter dan status saat ini
 *  save            : Simpan parameter ke EEPROM / Flash
 *  load            : Muat parameter dari EEPROM / Flash
 *  reset           : Kembalikan parameter ke nilai default (belum disimpan)
 *  state           : Tampilkan state sistem dan state navigasi saat ini
 *  sn              : Sensor stream: alirkan pembacaan L/F/R tiap 200 ms 
 *                    (Hanya tampil di mode MAINT)
 *  mt              : Motor test: tes pergerakan motor kiri & kanan (maju & mundur)
 *  rc              : Reset counter: nolkan hitungan percobaan dan retry
 *  go              : Mulai jalan robot dari serial
 *  stop            : Hentikan robot dari serial
 *  side=l/r        : Pilih track lintasan (l = kiri, r = kanan)
 *  cp=0..3         : Pilih titik mulai (0 = START, 1-3 = RETRY)
 *  cal             : Kalibrasi kecepatan ( robot jalan lurus )
 *  cald=<mm>       : Masukkan jarak tempuh hasil ukur nyata setelah kalibrasi
 *
 * ----------------------------------------------------------------------------
 * 2. PARAMETER KONTROL & NAVIGASI (Format pengubahan: nama=nilai)
 * ----------------------------------------------------------------------------
 *  Singkatan  Kepanjangan           Arti / Fungsi
 *  ---------------------------------------------------------------------------
 *  kp         Proportional          Penguat PID: Koreksi per mm error
 *  ki         Integral              Penguat PID: Akumulasi error terhadap waktu
 *  kd         Derivative            Penguat PID: Meredam ayunan / lonjakan
 *  mc         Max Correction        Batas nilai koreksi PID (skala PWM)
 *  bs         Base Speed            Kecepatan jelajah utama (PWM 0-255)
 *  ms         Max Speed             Batas kecepatan maksimum keras (PWM)
 *  ts         Turn Speed            Kecepatan robot saat bermanuver / belok
 *  mn         Min Speed             Kecepatan terendah saat mendekati dinding depan
 *  tg         Target Side           Jarak ideal ke dinding samping yang diikuti (mm)
 *  op         Open Side             Di atas nilai ini, dinding dianggap hilang (tikungan)
 *  ft         Front Turn            Di bawah nilai ini, robot wajib melakukan belokan
 *  fs         Front Slow            Di bawah nilai ini, robot mulai melambat/mengerem
 *  fc         Front Clear           Pivot selesai jika bagian depan sudah lebih lega dari nilai ini
 *  sg         Sensor Gap            Jeda antar pemanggilan sensor (µs)
 *  cy         Cycle                 Periode minimum satu siklus baca 3 sensor (ms)
 *  vf         Velocity Full         Kecepatan fisik roda (mm/s) pada PWM 255
 *  bd         Brake Deceleration    Tingkat perlambatan saat PWM diturunkan (mm/s²)
 *  lag        Lag                   Tunda mekanik motor ditambah filter respon (ms)
 *  sa         Sensor-Axle           Jarak posisi sensor samping ke as roda (mm)
 *  arc        Arc Percent           Kecepatan roda bagian dalam saat melengkung (% dari ts)
 *  adv        Advance               Batas waktu maju sebelum mulai melengkung (ms)
 *  rec        Recover               Durasi mundur saat robot terdeteksi macet (ms)
 *  tto        Turn Timeout          Batas waktu maksimal satu manuver belok (ms)
 *  stk        Stuck                 Batas waktu tanpa perubahan sensor sebelum dianggap macet (ms)
 *  fin        Finish Detect         Aktifkan deteksi finish (1 = Aktif, 0 = Nonaktif)
 *  fside      Finish Side           Kiri & kanan di atas nilai ini berarti robot keluar arena (mm)
 *  fhold      Finish Hold           Syarat finish harus bertahan terus selama durasi ini (ms)
 *  rto        Run Timeout           Batas waktu maksimal satu run lintasan (ms)
 *  tel        Telemetry             Aktifkan log data (1 = Cetak log tiap 100 ms saat jalan)
 *
 * ----------------------------------------------------------------------------
 * 3. KETERANGAN OUTPUT PERINTAH 'SHOW'
 * ----------------------------------------------------------------------------
 *  Contoh Baris Hasil                Arti / Penjelasan
 *  ---------------------------------------------------------------------------
 *  side=l cp=0 state=ERROR           Track kiri, start di CP 0, status sistem ERROR,
 *  sensor=GAGAL                      dan sensor gagal saat tes awal.
 *
 *  v@bs=475 mm/s                     Perkiraan kecepatan fisik robot saat berjalan 
 *                                    pada Base Speed (bs).
 *
 *  mulai melambat di F=260 mm        Jarak depan (Front) di mana robot mulai 
 *                                    melakukan pengereman.
 *
 *  pivot90=215 ms                    Perkiraan durasi waktu yang dibutuhkan untuk 
 *                                    putar di tempat sebesar 90°.
 *
 *  siklus=25.0 ms                    Durasi waktu nyata untuk menyelesaikan satu 
 *                                    siklus pembacaan sensor.
 *
 *  percobaan=0 retry=0/3             Hitungan jumlah run yang telah dilakukan dan 
 *                                    sisa jatah retry yang tersisa (misal: 0 dari 3).
 * ============================================================================
 */

#include <Arduino.h>
#include <Preferences.h>
#include "wall_follower_types.h"

// =====================================================================
// 1. KONFIGURASI PIN
// =====================================================================
// HC-SR04 (urutan array: kiri, depan, kanan)
const uint8_t PIN_US_TRIG[3] = { 26, 25, 27 };  // output
const uint8_t PIN_US_ECHO[3] = { 34, 35, 13 };  // input; WAJIB lewat pembagi tegangan 5V -> 3.3V

const uint8_t PIN_AIN1 = 19;    // DRV8833 IN1 (motor A / kiri)
const uint8_t PIN_AIN2 = 18;    // DRV8833 IN2
const uint8_t PIN_BIN1 = 33;    // DRV8833 IN3 (motor B / kanan)
const uint8_t PIN_BIN2 = 32;    // DRV8833 IN4
const uint8_t PIN_NSLEEP = 17;  // DRV8833 EEP/nSLEEP: HIGH = driver aktif

const uint8_t PIN_BTN_LEFT = 23;
const uint8_t PIN_BTN_RIGHT = 22;
const uint8_t PIN_LED_RED = 4;
const uint8_t PIN_LED_GREEN = 21;

// =====================================================================
// 2. KONSTANTA HARDWARE (jarang diubah)
// =====================================================================
const uint32_t PWM_FREQ = 20000;  // 20 kHz: senyap, riak arus kecil, DRV8833 mendukung
const uint8_t PWM_RES = 8;        // 0..255
const int PWM_MAX = 255;

// Ubah true jika arah putar motor terbalik karena pengkabelan
const bool MOTOR_L_INVERT = false;
const bool MOTOR_R_INVERT = false;
// true = tukar peran motor A dan B (jika kabel OUT1/OUT2 ternyata di sisi kanan robot)
const bool MOTOR_SWAP_LR = true;
// Kompensasi motor tidak identik (1.00 = tanpa koreksi)
const float MOTOR_L_GAIN = 1.00f;
const float MOTOR_R_GAIN = 1.00f;

// Slew rate dinyatakan per SLEW_REF_MS, lalu diskalakan dengan waktu nyata antar perintah
// sehingga hasilnya sama walau waktu siklus sensor berubah.
const float SLEW_REF_MS = 25.0f;
const int SLEW_UP = 22;    // batas kenaikan PWM per SLEW_REF_MS (anti selip saat start)
const int SLEW_DOWN = 80;  // batas penurunan PWM per SLEW_REF_MS

// Sensor ultrasonik HC-SR04
const uint16_t US_MAX_RANGE_MM = 900;   // mm, tanpa echo / lebih jauh dianggap "jauh"
const uint16_t US_MIN_RANGE_MM = 20;    // batas bawah fisik HC-SR04 (~2 cm)
const uint32_t US_TIMEOUT_US = 5500;    // batas tunggu echo (~0,94 m pulang-pergi)
const uint8_t US_TRIG_PULSE_US = 10;    // lebar pulsa trigger sesuai datasheet
const uint16_t SENSOR_RELEASE_MS = 45;  // HC-SR04 menahan ECHO sampai ~38 ms jika tanpa echo
const float SIDE_ALPHA = 0.6f;          // bobot EMA sensor samping (makin kecil makin halus)
const int STUCK_DELTA_MM = 15;          // toleransi derau untuk deteksi macet
// true  = ERROR bila ada sensor tanpa echo saat boot, dan start ditolak bila semua sensor buta
// false = lewati pemeriksaan (berguna saat uji di meja tanpa dinding)
const bool REQUIRE_SENSOR_SELFTEST = true;

// Model gerak (dipakai untuk menyelaraskan sensor dengan kecepatan roda)
const float TRACK_WIDTH_MM = 118.0f;    // jarak antar titik sentuh roda kiri-kanan
const int RUNWAY_MARGIN_MM = 15;        // cadangan jarak pengereman
const uint8_t PIVOT_CONFIRM_LOOPS = 2;  // pembacaan "depan lega" berturut-turut untuk menghentikan pivot
const uint32_t CAL_RUN_MS = 1500;       // durasi lari kalibrasi kecepatan
const uint32_t CAL_RAMP_MS = 120;       // perkiraan waktu akselerasi (dikeluarkan dari hitungan)

const uint8_t MAX_RETRY = 3;  // jatah retry sesuai aturan lomba

const uint32_t DEBOUNCE_MS = 30;
const uint32_t SELECT_WINDOW_MS = 1200;
const uint32_t START_DELAY_MS = 400;
const uint32_t FINISH_GRACE_MS = 1500;  // deteksi finish diabaikan di awal
const uint32_t OPEN_GRACE_MS = 250;     // abaikan "dinding hilang" setelah belokan

// =====================================================================
// 3. PARAMETER TUNING (disimpan ke flash dengan perintah "save")
// =====================================================================
const uint16_t PARAMS_VER = 5;  // dinaikkan karena struktur parameter berubah

struct Params {
  uint16_t ver = PARAMS_VER;
  // PID
  float kp = 0.60f;  // PWM per mm error
  float ki = 0.00f;  // biasanya 0
  float kd = 0.02f;  // PWM per (mm/detik), kecil karena derau HC-SR04
  int maxCorr = 60;  // batas koreksi PID (PWM)
  // Kecepatan (PWM 0..255). Awal tuning dibuat konservatif.
  int baseSpeed = 110;
  int maxSpeed = 175;  // batas keras (~70% dari 8,4 V untuk N20 6 V)
  int turnSpeed = 100;
  int minSpeed = 60;  // di bawah ini N20 sulit berputar
  // Jarak (mm, dari muka sensor)
  int targetSide = 110;  // jarak ideal ke dinding yang diikuti
  int openSide = 250;    // di atas ini dinding dianggap hilang
  int frontTurn = 120;   // di bawah ini wajib belok
  int frontSlow = 260;   // di bawah ini mulai melambat (dinaikkan otomatis jika kecepatan tinggi)
  int frontClear = 260;  // pivot selesai jika depan sudah lega
  // Sensor & sinkronisasi dengan kecepatan roda
  uint16_t usGapUs = 2500;         // jeda antar sensor (us), cegah crosstalk
  uint16_t cycMs = 25;             // periode minimum satu siklus baca 3 sensor (ms)
  float speedFullMmS = 1100.0f;    // kecepatan roda (mm/s) pada PWM 255, kalibrasi dengan "cal"
  float brakeDecelMmS2 = 3000.0f;  // perlambatan realistis saat PWM diturunkan (mm/s^2)
  uint16_t lagMs = 40;             // tunda mekanik motor + filter (ms)
  int sensorAxleMm = 68;           // jarak sensor samping ke as roda (mm)
  // Manuver
  int arcPct = 35;                // kecepatan roda dalam saat arc (% turnSpeed)
  uint16_t outerAdvanceMs = 350;  // batas maksimum maju sebelum arc (waktu dihitung dari kecepatan)
  uint16_t recoverMs = 300;       // durasi mundur saat recovery
  uint16_t turnTimeout = 2200;    // batas waktu satu manuver
  uint16_t stuckMs = 2500;        // dianggap macet jika tidak ada perubahan
  int stuckFrontMax = 500;        // cek macet hanya jika depan lebih dekat dari ini
  // Finish & batas waktu
  bool finishDetect = true;
  int finishSide = 700;         // kiri DAN kanan di atas ini = keluar arena
  uint16_t finishHold = 350;    // harus bertahan selama ini (ms)
  uint32_t runTimeout = 90000;  // berhenti otomatis (ms)
  // Debug
  bool telemetry = false;
};

Params P;
Preferences prefs;

// Koreksi nilai yang tidak masuk akal agar tidak ada celah konfigurasi (dipanggil sebelum run/save/load)
void sanitizeParams() {
  P.maxSpeed = constrain(P.maxSpeed, 30, 255);
  P.baseSpeed = constrain(P.baseSpeed, 20, P.maxSpeed);
  P.turnSpeed = constrain(P.turnSpeed, 20, P.maxSpeed);
  P.minSpeed = constrain(P.minSpeed, 20, P.baseSpeed);
  P.maxCorr = constrain(P.maxCorr, 0, 150);
  P.targetSide = constrain(P.targetSide, 50, 250);
  P.frontTurn = constrain(P.frontTurn, 60, 400);
  if (P.frontSlow < P.frontTurn + 40) P.frontSlow = P.frontTurn + 40;
  if (P.frontClear < P.frontTurn + 20) P.frontClear = P.frontTurn + 20;
  P.openSide = constrain(P.openSide, P.targetSide + 40, (int)US_MAX_RANGE_MM - 50);
  P.finishSide = constrain(P.finishSide, P.openSide + 50, (int)US_MAX_RANGE_MM);
  P.cycMs = constrain(P.cycMs, 15, 100);
  P.usGapUs = constrain(P.usGapUs, 0, 20000);
  if (P.speedFullMmS < 100.0f) P.speedFullMmS = 100.0f;
  if (P.brakeDecelMmS2 < 200.0f) P.brakeDecelMmS2 = 200.0f;
  P.lagMs = constrain(P.lagMs, 0, 300);
  P.sensorAxleMm = constrain(P.sensorAxleMm, 0, 200);
  P.arcPct = constrain(P.arcPct, 0, 100);
  if (P.turnTimeout < 500) P.turnTimeout = 500;
  if (P.stuckMs < 800) P.stuckMs = 800;
  if (P.runTimeout < 5000) P.runTimeout = 5000;
}

// =====================================================================
// 4. KONFIGURASI CHECKPOINT (START + 3 titik RETRY)
// =====================================================================
// followOverride : -1 = ikut aturan track, 0 = paksa dinding kiri, 1 = paksa dinding kanan
// startTurn      : -1 = pivot kiri, 0 = tidak, 1 = pivot kanan (sebelum mulai)
// startTurnMs    : lama pivot awal
// speedPct       : skala kecepatan (%) untuk titik ini
// Robot tanpa memori, jadi dari titik mana pun cukup meraba dinding lalu mengikutinya.
// Jika arah hadap robot di titik retry diketahui (ditentukan wasit), isi startTurn/startTurnMs
// setelah diuji di arena sebenarnya (waktu pivot 90 derajat = lihat "show").
struct CheckpointCfg {
  const char* name;
  int8_t followOverride;
  int8_t startTurn;
  uint16_t startTurnMs;
  uint8_t speedPct;
};

const CheckpointCfg CP_CFG[4] = {
  { "START", -1, 0, 0, 100 },
  { "RETRY-1", -1, 0, 0, 90 },
  { "RETRY-2", -1, 0, 0, 90 },
  { "RETRY-3", -1, 0, 0, 90 }
};

// Track KIRI mengikuti dinding KANAN (dinding tengah) dan sebaliknya.
// Verifikasi dengan arena tiruan, ubah ke false jika ternyata sebaliknya.
const bool LEFT_TRACK_FOLLOWS_RIGHT_WALL = true;

// =====================================================================
// 5. TIPE DATA & STATE
// =====================================================================
enum SensorIdx : uint8_t { SENS_L = 0,
                           SENS_F = 1,
                           SENS_R = 2 };
const char* SENS_NAMES[] = { "KIRI", "DEPAN", "KANAN" };

// enum SysState : uint8_t {
//   SYS_IDLE, SYS_READY, SYS_RUN, SYS_FINISHED, SYS_ERROR, SYS_MAINT
// };
const char* SYS_NAMES[] = { "IDLE", "READY", "RUN", "FINISHED", "ERROR", "MAINT" };

// enum NavState : uint8_t {
//   NAV_START_TURN, NAV_ACQUIRE, NAV_FOLLOW, NAV_TURN_OUTER,
//   NAV_PIVOT, NAV_RECOVER
// };
const char* NAV_NAMES[] = { "START_TURN", "ACQUIRE", "FOLLOW", "TURN_OUTER",
                            "PIVOT", "RECOVER" };

// enum LedMode : uint8_t { LED_OFF, LED_ON, LED_BLINK_SLOW, LED_BLINK_SLOW_INV, LED_BLINK_FAST };

SysState sys = SYS_IDLE;
NavState nav = NAV_FOLLOW;

bool sensorsReady = false;
uint16_t dist[3] = { US_MAX_RANGE_MM, US_MAX_RANGE_MM, US_MAX_RANGE_MM };
// Riwayat 3 pembacaan mentah per sensor untuk median filter (membuang lonjakan palsu HC-SR04)
uint16_t rawHist[3][3] = {
  { US_MAX_RANGE_MM, US_MAX_RANGE_MM, US_MAX_RANGE_MM },
  { US_MAX_RANGE_MM, US_MAX_RANGE_MM, US_MAX_RANGE_MM },
  { US_MAX_RANGE_MM, US_MAX_RANGE_MM, US_MAX_RANGE_MM }
};
uint8_t histIdx[3] = { 0, 0, 0 };
uint8_t noEchoCnt[3] = { 0, 0, 0 };  // jumlah pembacaan berturut-turut tanpa echo (diagnostik)

// Pilihan operator
bool trackRight = false;  // false = track kiri, true = track kanan
uint8_t selCp = 0;        // 0..3
bool followRight = true;  // dinding yang diikuti
uint8_t cpSpeedPct = 100;

// Navigasi
uint32_t navT0 = 0, runStartMs = 0, finishTimer = 0, graceUntil = 0;
int pivotDir = 1;
uint8_t openCount = 0;
uint8_t pivotClearCnt = 0;
uint16_t outerAdvMsCur = 120;  // lama maju sebelum arc, dihitung dari kecepatan saat ini
int lastErr = 0, lastCorr = 0;
int lastSig = 0;
uint32_t lastSigT = 0;

// Penyelarasan sensor <-> kecepatan
float loopMsEma = 25.0f;  // rata-rata waktu siklus nyata saat RUN
uint32_t runLastUs = 0;
uint32_t lastSenseUs = 0;
int calPwmUsed = 0;

// Penghitung percobaan (RAM, hilang saat restart)
uint8_t runCount = 0, retryCount = 0;

// PID
float pidI = 0, pidPrevErr = 0;
uint32_t pidPrevT = 0;
bool pidFirst = true;

// Motor
int curL = 0, curR = 0, cmdL = 0, cmdR = 0;
uint32_t slewLastUs = 0;
float slewScale = 1.0f;

// LED
LedMode redMode = LED_OFF, greenMode = LED_OFF;

// Tombol
// struct Button {
//   uint8_t  pin;
//   bool     down = false, rawLast = false, evPress = false, evRelease = false;
//   uint32_t tChange = 0, tDown = 0, lastHold = 0;
//   explicit Button(uint8_t p) : pin(p) {}
// };
Button btnL(PIN_BTN_LEFT), btnR(PIN_BTN_RIGHT);

// Pemilihan
uint8_t pressCount = 0;
uint32_t lastPressT = 0;
bool selRight = false;

// Maintenance
bool streamSensors = false;

// =====================================================================
// 6. LED
// =====================================================================
void ledApply(uint8_t pin, LedMode m) {
  uint32_t t = millis();
  bool v = false;
  switch (m) {
    case LED_ON: v = true; break;
    case LED_BLINK_SLOW: v = ((t / 500) % 2) == 0; break;
    case LED_BLINK_SLOW_INV: v = ((t / 500) % 2) != 0; break;
    case LED_BLINK_FAST: v = ((t / 120) % 2) == 0; break;
    default: v = false;
  }
  digitalWrite(pin, v ? HIGH : LOW);
}

void ledsSet(LedMode r, LedMode g) {
  redMode = r;
  greenMode = g;
}
void ledsUpdate() {
  ledApply(PIN_LED_RED, redMode);
  ledApply(PIN_LED_GREEN, greenMode);
}

// Berkedip n kali (blocking, hanya dipakai saat robot diam)
void blinkN(uint8_t pin, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    digitalWrite(pin, HIGH);
    delay(180);
    digitalWrite(pin, LOW);
    delay(180);
  }
  delay(250);
}

// =====================================================================
// 7. TOMBOL
// =====================================================================
void btnUpdate(Button& b) {
  b.evPress = b.evRelease = false;
  bool raw = (digitalRead(b.pin) == LOW);  // aktif LOW (INPUT_PULLUP)
  uint32_t now = millis();
  if (raw != b.rawLast) {
    b.rawLast = raw;
    b.tChange = now;
  }
  if ((now - b.tChange) >= DEBOUNCE_MS && raw != b.down) {
    b.down = raw;
    if (raw) {
      b.evPress = true;
      b.tDown = now;
    } else {
      b.evRelease = true;
      b.lastHold = now - b.tDown;
    }
  }
}

// =====================================================================
// 8. MOTOR DRIVER DRV8833 (PWM di pin IN, slow decay)
// =====================================================================
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
  digitalWrite(PIN_NSLEEP, LOW);  // driver mati saat boot
  ledcAttach(PIN_AIN1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_AIN2, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_BIN1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_BIN2, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_AIN1, 0);
  ledcWrite(PIN_AIN2, 0);
  ledcWrite(PIN_BIN1, 0);
  ledcWrite(PIN_BIN2, 0);
}

// Batas perubahan PWM per panggilan, diskalakan oleh slewScale (waktu nyata / SLEW_REF_MS)
int slew(int cur, int tgt) {
  int up = max(1, (int)(SLEW_UP * slewScale));
  int dn = max(1, (int)(SLEW_DOWN * slewScale));
  int d = constrain(tgt - cur, -dn, up);
  return cur + d;
}

// Perintah kecepatan kiri/kanan (-255..255)
void setMotors(int l, int r) {
  uint32_t nowUs = micros();
  float dtMs = (slewLastUs == 0) ? SLEW_REF_MS : (nowUs - slewLastUs) / 1000.0f;
  slewLastUs = nowUs;
  dtMs = constrain(dtMs, 1.0f, 60.0f);
  slewScale = dtMs / SLEW_REF_MS;

  digitalWrite(PIN_NSLEEP, HIGH);
  cmdL = l;
  cmdR = r;
  curL = slew(curL, l);
  curR = slew(curR, r);
  int outL = (int)(curL * MOTOR_L_GAIN) * (MOTOR_L_INVERT ? -1 : 1);
  int outR = (int)(curR * MOTOR_R_GAIN) * (MOTOR_R_INVERT ? -1 : 1);
  if (MOTOR_SWAP_LR) {
    int tmp = outL;
    outL = outR;
    outR = tmp;
  }
  motorWrite(PIN_AIN1, PIN_AIN2, outL);
  motorWrite(PIN_BIN1, PIN_BIN2, outR);
}

void motorsBrake() {
  curL = curR = cmdL = cmdR = 0;
  slewLastUs = 0;
  motorWrite(PIN_AIN1, PIN_AIN2, 0);
  motorWrite(PIN_BIN1, PIN_BIN2, 0);
}

void motorsOff() {
  motorsBrake();
  ledcWrite(PIN_AIN1, 0);
  ledcWrite(PIN_AIN2, 0);
  ledcWrite(PIN_BIN1, 0);
  ledcWrite(PIN_BIN2, 0);
  digitalWrite(PIN_NSLEEP, LOW);
}

// dir = +1 putar kanan (searah jarum jam), -1 putar kiri, di tempat
void pivotMotors(int dir, int spd) {
  setMotors(dir * spd, -dir * spd);
}

// Jalankan motor selama ms, setMotors dipanggil berulang agar slew rate sempat naik
void driveFor(int l, int r, uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    setMotors(l, r);
    delay(10);
  }
}

// =====================================================================
// 9. SENSOR HC-SR04 (3 unit, dibaca bergantian)
// =====================================================================
// Satu pengukuran. Return false jika tidak ada echo (anggap "jauh") atau ECHO masih
// tertahan HIGH dari siklus sebelumnya (sensor belum siap dipicu ulang).
bool usMeasure(uint8_t i, uint16_t& mm) {
  const uint8_t echo = PIN_US_ECHO[i];
  if (digitalRead(echo) == HIGH) return false;

  digitalWrite(PIN_US_TRIG[i], LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_US_TRIG[i], HIGH);
  delayMicroseconds(US_TRIG_PULSE_US);
  digitalWrite(PIN_US_TRIG[i], LOW);

  uint32_t d = pulseIn(echo, HIGH, US_TIMEOUT_US);  // 0 = timeout
  if (d == 0) return false;

  uint32_t v = d * 343UL / 2000UL;  // mm = us * 0.343 mm/us / 2 (pulang-pergi)
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
  delay(100);

  // Self-test: tiap sensor harus memberi minimal satu echo valid dalam 5 percobaan
  bool ok = true;
  for (uint8_t i = 0; i < 3; i++) {
    if (digitalRead(PIN_US_ECHO[i]) == HIGH)
      Serial.printf("[US] Sensor %s: pin ECHO tertahan HIGH (cek kabel/pembagi tegangan)\n", SENS_NAMES[i]);
    bool alive = false;
    uint16_t mm = 0;
    for (uint8_t t = 0; t < 5 && !alive; t++) {
      alive = usMeasure(i, mm);
      delay(SENSOR_RELEASE_MS);
    }
    if (alive) Serial.printf("[US] Sensor %s OK (%u mm)\n", SENS_NAMES[i], mm);
    else {
      Serial.printf("[US] Sensor %s TIDAK ADA ECHO (cek VCC 5V/kabel, atau tidak ada dinding < 0,9 m)\n", SENS_NAMES[i]);
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
    uint16_t mm = US_MAX_RANGE_MM;
    if (usMeasure(i, mm)) {
      noEchoCnt[i] = 0;
    } else {
      mm = US_MAX_RANGE_MM;
      if (noEchoCnt[i] < 255) noEchoCnt[i]++;
    }  // tanpa echo = jauh

    rawHist[i][histIdx[i]] = mm;
    histIdx[i] = (histIdx[i] + 1) % 3;
    uint16_t m = median3(rawHist[i][0], rawHist[i][1], rawHist[i][2]);

    if (i == SENS_F) dist[i] = m;                                               // depan: median saja (cepat)
    else dist[i] = (uint16_t)(SIDE_ALPHA * m + (1.0f - SIDE_ALPHA) * dist[i]);  // samping: EMA

    delayMicroseconds(P.usGapUs);  // beri waktu gema mereda sebelum sensor berikutnya
  }
}

// Isi ulang riwayat sebelum mulai jalan agar tidak memakai data lama (robot diam, aman memakai delay)
void sensorsPrime() {
  for (uint8_t k = 0; k < 3; k++) {
    readSensors();
    delay(SENSOR_RELEASE_MS);
  }
  for (uint8_t i = 0; i < 3; i++) dist[i] = median3(rawHist[i][0], rawHist[i][1], rawHist[i][2]);
}

// Tahan siklus agar tiap sensor dipicu ulang paling cepat setiap P.cycMs (anti gema)
// sekaligus membuat dt PID dan model kecepatan stabil.
void waitSensorCycle() {
  if (lastSenseUs != 0) {
    uint32_t elapsedMs = (micros() - lastSenseUs) / 1000UL;
    if (elapsedMs < P.cycMs) delay(P.cycMs - elapsedMs);
  }
  lastSenseUs = micros();
}

inline int followDist() {
  return followRight ? dist[SENS_R] : dist[SENS_L];
}

// =====================================================================
// 9b. MODEL KECEPATAN (menyelaraskan sensor dengan putaran roda)
// =====================================================================
// Kecepatan roda hasil estimasi dari PWM (kalibrasi dengan perintah "cal")
float speedMmS(int pwm) {
  return (float)abs(pwm) * P.speedFullMmS / 255.0f;
}

// Kecepatan maju robot saat ini (dari PWM aktual setelah slew), 0 jika diam/mundur
float robotSpeedMmS() {
  int fwd = (curL + curR) / 2;
  return (fwd > 0) ? speedMmS(fwd) : 0.0f;
}

// Total tunda dari kejadian nyata sampai motor bereaksi: sensor + filter median/EMA + mekanik
float latencyMs() {
  return loopMsEma * 2.0f + (float)P.lagMs;
}

// Jarak yang ditempuh robot selama tunda + pengereman dari PWM fromPwm ke toPwm
int brakeRunwayMm(int fromPwm, int toPwm) {
  float v1 = speedMmS(fromPwm), v2 = speedMmS(toPwm);
  float a = (P.brakeDecelMmS2 < 200.0f) ? 200.0f : P.brakeDecelMmS2;
  float runway = v1 * latencyMs() / 1000.0f;
  if (v1 > v2) runway += (v1 * v1 - v2 * v2) / (2.0f * a);
  return (int)runway + RUNWAY_MARGIN_MM;
}

// Jarak depan tempat robot mulai melambat. Tidak pernah lebih kecil dari P.frontSlow,
// dan otomatis diperlebar bila kecepatan jelajah dinaikkan.
int dynFrontSlow(int cruisePwm, int lowPwm) {
  int need = P.frontTurn + brakeRunwayMm(cruisePwm, lowPwm);
  return max(P.frontSlow, need);
}

// Lama maju setelah sensor samping kehilangan dinding, sebelum mulai arc.
// Sebagian jarak sudah ditempuh saat sensor "menunda", jadi dikurangkan.
uint16_t advanceMsForSpeed() {
  float vNow = robotSpeedMmS();
  float vAdv = 0.5f * (vNow + speedMmS(P.turnSpeed));
  if (vAdv < 30.0f) return P.outerAdvanceMs;
  float distMm = (float)P.sensorAxleMm - vNow * latencyMs() / 1000.0f;
  if (distMm <= 0.0f) return 0;
  float ms = distMm / vAdv * 1000.0f;
  if (ms > (float)P.outerAdvanceMs) ms = (float)P.outerAdvanceMs;
  return (uint16_t)ms;
}

// Kecepatan pivot: penuh di awal, melambat setelah ~60% perkiraan waktu belok 90 derajat
// agar tidak kebablasan (sensor ultrasonik tidak secepat putaran roda).
int pivotSpeedFor(uint32_t tMs) {
  float w = 2.0f * speedMmS(P.turnSpeed) / TRACK_WIDTH_MM;  // rad/s
  if (w < 0.1f) return P.turnSpeed;
  float t90 = (1.5708f / w) * 1000.0f;  // ms
  if ((float)tMs > 0.6f * t90) {
    int slow = max(P.minSpeed, (int)(P.turnSpeed * 0.65f));
    return min(P.turnSpeed, slow);
  }
  return P.turnSpeed;
}

// map() aman: tidak membagi nol jika batas masukan sama
int mapSafe(int x, int inLo, int inHi, int outLo, int outHi) {
  if (inHi <= inLo) return outLo;
  return outLo + (int)((long)(x - inLo) * (outHi - outLo) / (inHi - inLo));
}

// Pembaruan waktu siklus nyata saat RUN
void runTimingUpdate() {
  uint32_t nowUs = micros();
  if (runLastUs != 0) {
    float ms = (nowUs - runLastUs) / 1000.0f;
    if (ms > 200.0f) ms = 200.0f;
    loopMsEma = 0.9f * loopMsEma + 0.1f * ms;
  }
  runLastUs = nowUs;
}

// =====================================================================
// 10. PID
// =====================================================================
void pidReset() {
  pidI = 0;
  pidPrevErr = 0;
  pidFirst = true;
}

int pidCompute(int err) {
  uint32_t now = millis();
  float dt = (now - pidPrevT) / 1000.0f;
  if (pidFirst || dt <= 0.0f || dt > 0.2f) {
    dt = 0.02f;
    pidPrevErr = err;
    pidFirst = false;
  }
  pidI += err * dt;
  float lim = (P.ki > 0.0f) ? (P.maxCorr / P.ki) : 0.0f;  // anti-windup
  pidI = constrain(pidI, -lim, lim);
  float d = (err - pidPrevErr) / dt;
  pidPrevErr = err;
  pidPrevT = now;
  float out = P.kp * err + P.ki * pidI + P.kd * d;
  return constrain((int)out, -P.maxCorr, P.maxCorr);
}

// =====================================================================
// 11. NAVIGASI (state machine)
// =====================================================================
void navEnter(NavState s) {
  nav = s;
  navT0 = millis();
  lastSig = dist[0] + dist[1] + dist[2];
  lastSigT = navT0;
  pivotClearCnt = 0;
  if (s == NAV_FOLLOW) {
    pidReset();
    openCount = 0;
    graceUntil = navT0 + OPEN_GRACE_MS;
  }
  if (s == NAV_TURN_OUTER) outerAdvMsCur = advanceMsForSpeed();
  if (P.telemetry) Serial.printf("[NAV] -> %s\n", NAV_NAMES[s]);
}

NavState decideAfterStart() {
  return (followDist() > P.openSide && dist[SENS_F] > P.frontTurn) ? NAV_ACQUIRE : NAV_FOLLOW;
}

// Macet jika semua pembacaan nyaris tidak berubah padahal depan dekat
bool isStuck(uint32_t now) {
  int sum = dist[0] + dist[1] + dist[2];
  if (abs(sum - lastSig) > STUCK_DELTA_MM || dist[SENS_F] > P.stuckFrontMax) {
    lastSig = sum;
    lastSigT = now;
  }
  return (now - lastSigT) > P.stuckMs;
}

void navStep() {
  const uint32_t now = millis();
  const int F = dist[SENS_F];
  const int S = followDist();
  const int dir = followRight ? 1 : -1;  // +1: dinding di kanan
  const int towardDir = dir;             // pivot ke arah dinding
  const int awayDir = -dir;              // pivot menjauhi dinding
  const int baseS = P.baseSpeed * cpSpeedPct / 100;

  switch (nav) {

    // Pivot awal paksa (opsional per checkpoint)
    case NAV_START_TURN:
      {
        const CheckpointCfg& c = CP_CFG[selCp];
        if (now - navT0 >= c.startTurnMs) navEnter(decideAfterStart());
        else pivotMotors(c.startTurn, P.turnSpeed);
        break;
      }

    // Belum ada dinding di sisi yang diikuti: maju sambil sedikit mengarah ke sana
    case NAV_ACQUIRE:
      {
        setMotors(P.turnSpeed + dir * 10, P.turnSpeed - dir * 10);
        if (F < P.frontTurn) {
          pivotDir = (S > P.openSide) ? towardDir : awayDir;
          navEnter(NAV_PIVOT);
        } else if (S < P.openSide) navEnter(NAV_FOLLOW);
        else if (now - navT0 > 3000) navEnter(NAV_FOLLOW);
        else if (isStuck(now)) navEnter(NAV_RECOVER);
        break;
      }

    // Ikuti dinding dengan PID
    case NAV_FOLLOW:
      {
        if (F < P.frontTurn) {
          // Depan tertutup: jika sisi ikut terbuka belok ke sana, selain itu menjauh (juga U-turn)
          pivotDir = (S > P.openSide) ? towardDir : awayDir;
          navEnter(NAV_PIVOT);
          break;
        }
        if (now > graceUntil && S > P.openSide) {
          if (++openCount >= 2) {
            navEnter(NAV_TURN_OUTER);
            break;
          }
        } else {
          openCount = 0;
        }
        if (isStuck(now)) {
          navEnter(NAV_RECOVER);
          break;
        }

        int err = constrain(S - P.targetSide, -150, 150);
        int corr = pidCompute(err);
        lastErr = err;
        lastCorr = corr;

        // Profil kecepatan: melambat mendekati dinding depan.
        // Titik mulai melambat dihitung dari kecepatan roda + tunda sensor (lihat dynFrontSlow).
        int s = baseS;
        int sLow = min(P.minSpeed, baseS);
        int slowAt = dynFrontSlow(baseS, sLow);
        if (F < slowAt) {
          s = mapSafe(F, P.frontTurn, slowAt, sLow, baseS);
          s = constrain(s, sLow, baseS);
        }
        // Error positif = terlalu jauh dari dinding, arahkan ke dinding
        int l = constrain(s + dir * corr, 0, P.maxSpeed);
        int r = constrain(s - dir * corr, 0, P.maxSpeed);
        setMotors(l, r);
        break;
      }

    // Dinding hilang (tikungan luar): maju sedikit lalu arc ke arah dinding
    case NAV_TURN_OUTER:
      {
        uint32_t t = now - navT0;
        if (t < outerAdvMsCur) {
          setMotors(P.turnSpeed, P.turnSpeed);
        } else {
          int inner = P.turnSpeed * P.arcPct / 100;
          if (followRight) setMotors(P.turnSpeed, inner);
          else setMotors(inner, P.turnSpeed);
          if (F < P.frontTurn) navEnter(NAV_FOLLOW);
          else if (t > (uint32_t)outerAdvMsCur + 120 && S < P.openSide) navEnter(NAV_FOLLOW);
          else if (t > P.turnTimeout) navEnter(NAV_ACQUIRE);
        }
        if (nav == NAV_TURN_OUTER && isStuck(now)) navEnter(NAV_RECOVER);
        break;
      }

    // Putar di tempat sampai depan lega (dikonfirmasi beberapa pembacaan berturut-turut)
    case NAV_PIVOT:
      {
        uint32_t t = now - navT0;
        pivotMotors(pivotDir, pivotSpeedFor(t));
        if (t > 150 && F >= P.frontClear) pivotClearCnt++;
        else pivotClearCnt = 0;
        if (pivotClearCnt >= PIVOT_CONFIRM_LOOPS) navEnter(NAV_FOLLOW);
        else if (t > P.turnTimeout) navEnter(NAV_RECOVER);
        break;
      }

    // Mundur sebentar lalu pivot menjauhi dinding
    case NAV_RECOVER:
      {
        setMotors(-P.turnSpeed, -P.turnSpeed);
        if (now - navT0 > P.recoverMs) {
          pivotDir = awayDir;
          navEnter(NAV_PIVOT);
        }
        break;
      }
  }
}

// =====================================================================
// 12. STATE SISTEM
// =====================================================================
void setSys(SysState s) {
  sys = s;
  Serial.printf("[SYS] %s\n", SYS_NAMES[s]);
  switch (s) {
    case SYS_IDLE:
      motorsOff();
      ledsSet(LED_BLINK_SLOW, LED_OFF);
      break;
    case SYS_READY:
      motorsOff();
      ledsSet(LED_OFF, LED_ON);
      break;
    case SYS_RUN: ledsSet(LED_OFF, LED_BLINK_FAST); break;
    case SYS_FINISHED:
      motorsBrake();
      ledsSet(LED_ON, LED_ON);
      break;
    case SYS_ERROR:
      motorsOff();
      ledsSet(LED_BLINK_FAST, LED_OFF);
      break;
    case SYS_MAINT:
      motorsOff();
      ledsSet(LED_BLINK_SLOW, LED_BLINK_SLOW_INV);
      break;
  }
}

// Terapkan pilihan track + checkpoint menjadi aturan dinding & kecepatan
void applySelection() {
  const CheckpointCfg& c = CP_CFG[selCp];
  if (c.followOverride >= 0) followRight = (c.followOverride == 1);
  else followRight = trackRight ? !LEFT_TRACK_FOLLOWS_RIGHT_WALL : LEFT_TRACK_FOLLOWS_RIGHT_WALL;
  cpSpeedPct = c.speedPct;
}

// Pemeriksaan sebelum jalan (penting untuk retry: robot baru diletakkan wasit/operator)
bool preStartCheck() {
  if (!REQUIRE_SENSOR_SELFTEST) return true;
  bool allFar = true, allNear = true;
  for (uint8_t i = 0; i < 3; i++) {
    if (dist[i] < US_MAX_RANGE_MM) allFar = false;
    if (dist[i] > 60) allNear = false;
  }
  if (allFar) {
    Serial.println("[PRE] Semua sensor tanpa echo (robot di udara / sensor lepas), start ditolak");
    return false;
  }
  if (allNear) {
    Serial.println("[PRE] Semua sensor < 6 cm (tertutup tangan?), start ditolak");
    return false;
  }
  return true;
}

void startRun() {
  if (!sensorsReady) {
    setSys(SYS_ERROR);
    return;
  }
  sanitizeParams();
  applySelection();
  Serial.printf("[RUN] track=%s titik=%s ikuti dinding=%s kecepatan=%u%%\n",
                trackRight ? "KANAN" : "KIRI", CP_CFG[selCp].name,
                followRight ? "KANAN" : "KIRI", cpSpeedPct);
  delay(START_DELAY_MS);
  sensorsPrime();
  if (!preStartCheck()) {
    blinkN(PIN_LED_RED, 3);
    setSys(SYS_READY);
    return;
  }

  runCount++;
  if (selCp > 0) {
    retryCount++;
    Serial.printf("[RETRY] pemakaian retry ke-%u dari %u\n", retryCount, MAX_RETRY);
    if (retryCount > MAX_RETRY) Serial.println("[WARN] Retry melebihi jatah aturan, konfirmasi ke wasit");
  }

  curL = curR = 0;
  loopMsEma = P.cycMs;  // nilai awal sebelum terukur
  runLastUs = 0;
  lastSenseUs = 0;
  slewLastUs = 0;
  runStartMs = millis();
  finishTimer = 0;
  pidReset();
  const CheckpointCfg& c = CP_CFG[selCp];
  if (c.startTurn != 0 && c.startTurnMs > 0) navEnter(NAV_START_TURN);
  else navEnter(decideAfterStart());
  setSys(SYS_RUN);
}

void endRun(const char* reason, bool success) {
  Serial.printf("[RUN] selesai: %s\n", reason);
  motorsBrake();
  if (!success) blinkN(PIN_LED_RED, 2);  // indikasi berhenti tanpa finish
  setSys(success ? SYS_FINISHED : SYS_IDLE);
}

void runStep() {
  waitSensorCycle();
  readSensors();
  runTimingUpdate();
  uint32_t now = millis();

  if (btnL.evPress || btnR.evPress) {
    endRun("berhenti manual", false);
    return;
  }
  if (now - runStartMs > P.runTimeout) {
    endRun("timeout", false);
    return;
  }

  // Deteksi finish: kiri DAN kanan sama-sama terbuka (keluar lorong) cukup lama
  if (P.finishDetect && nav != NAV_ACQUIRE && (now - runStartMs) > FINISH_GRACE_MS) {
    if (dist[SENS_L] >= P.finishSide && dist[SENS_R] >= P.finishSide && dist[SENS_F] > P.frontClear) {
      if (finishTimer == 0) finishTimer = now;
      else if (now - finishTimer >= P.finishHold) {
        endRun("finish terdeteksi", true);
        return;
      }
    } else {
      finishTimer = 0;
    }
  }

  navStep();

  // Merah ikut menyala saat RECOVER (tanda robot sempat macet)
  ledsSet(nav == NAV_RECOVER ? LED_ON : LED_OFF, LED_BLINK_FAST);

  if (P.telemetry) {
    static uint32_t lastPrint = 0;
    if (now - lastPrint >= 100) {
      lastPrint = now;
      Serial.printf("T,%lu,%s,L%d,F%d,R%d,err%d,corr%d,m%d/%d,lp%.0f,v%.0f\n", (unsigned long)(now - runStartMs),
                    NAV_NAMES[nav], dist[SENS_L], dist[SENS_F], dist[SENS_R], lastErr, lastCorr, cmdL, cmdR,
                    loopMsEma, robotSpeedMmS());
    }
  }
}

// IDLE: pilih track dengan jumlah tekan = titik mulai
void idleStep() {
  uint32_t now = millis();
  if ((btnL.evPress || btnR.evPress) && !(btnL.down && btnR.down)) {
    bool right = btnR.evPress;
    if (pressCount == 0 || right != selRight) {
      selRight = right;
      pressCount = 1;
    }  // sisi berbeda = hitung ulang
    else if (pressCount < 4)
      pressCount++;
    lastPressT = now;
    ledsSet(LED_ON, LED_OFF);  // merah tetap = sedang menghitung tekanan
  }
  if (pressCount > 0 && (now - lastPressT) > SELECT_WINDOW_MS) {
    trackRight = selRight;
    selCp = pressCount - 1;
    pressCount = 0;
    applySelection();
    Serial.printf("[SEL] track=%s titik=%s ikuti dinding=%s\n", trackRight ? "KANAN" : "KIRI",
                  CP_CFG[selCp].name, followRight ? "KANAN" : "KIRI");
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);
    blinkN(PIN_LED_RED, trackRight ? 2 : 1);
    blinkN(PIN_LED_GREEN, selCp + 1);
    setSys(SYS_READY);
  }
}

// READY: tombol SISI YANG SAMA dengan track terpilih (sebentar) = jalan, tahan >1 detik = pilih ulang
void readyStep() {
  bool relL = btnL.evRelease, relR = btnR.evRelease;
  if (!relL && !relR) return;
  uint32_t hold = relL ? btnL.lastHold : btnR.lastHold;
  if (hold >= 1000) {
    setSys(SYS_IDLE);
    return;
  }
  bool matches = (trackRight && relR) || (!trackRight && relL);
  if (matches) {
    startRun();
  } else {
    Serial.println("[READY] Tombol tidak sesuai track terpilih, diabaikan (tahan > 1 detik untuk pilih ulang)");
    blinkN(PIN_LED_RED, 3);
  }
}

// FINISHED / ERROR: tombol untuk lanjut
void finishedStep() {
  if (btnL.evRelease || btnR.evRelease) setSys(SYS_IDLE);
}
void errorStep() {
  if (btnL.evRelease || btnR.evRelease) ESP.restart();
}

// MAINT: KIRI = tes motor, KANAN = keluar
void motorTest() {
  Serial.println("[MAINT] Tes motor A(kiri) maju/mundur, lalu B(kanan) maju/mundur");
  int s = P.turnSpeed;
  driveFor(s, 0, 800);
  motorsBrake();
  delay(300);
  driveFor(-s, 0, 800);
  motorsBrake();
  delay(300);
  driveFor(0, s, 800);
  motorsBrake();
  delay(300);
  driveFor(0, -s, 800);
  motorsOff();
  Serial.println("[MAINT] Tes motor selesai");
}

void maintStep() {
  if (btnL.evPress) motorTest();
  if (btnR.evPress) {
    setSys(SYS_IDLE);
    return;
  }
  static uint32_t lastPrint = 0;
  if (streamSensors && millis() - lastPrint >= 200) {
    lastPrint = millis();
    readSensors();
    Serial.printf("L=%u F=%u R=%u mm\n", dist[SENS_L], dist[SENS_F], dist[SENS_R]);
  }
}

// Kalibrasi kecepatan: jalan lurus selama CAL_RUN_MS pada PWM baseSpeed, ukur jarak dengan meteran,
// lalu ketik "cald=<jarak mm>" agar speedFullMmS terisi otomatis.
void speedCalibrateRun() {
  if (!sensorsReady) {
    Serial.println("[CAL] sensor belum siap");
    return;
  }
  sanitizeParams();
  calPwmUsed = min(P.baseSpeed, P.maxSpeed);
  Serial.printf("[CAL] Siapkan lintasan lurus >= 1,5 m di lantai arena. Robot jalan %lu ms pada PWM %d dalam 2 detik...\n",
                (unsigned long)CAL_RUN_MS, calPwmUsed);
  delay(2000);
  sensorsPrime();
  bool aborted = false;
  uint32_t t0 = millis();
  lastSenseUs = 0;
  while (millis() - t0 < CAL_RUN_MS) {
    waitSensorCycle();
    readSensors();
    if (dist[SENS_F] < 200) {
      aborted = true;
      break;
    }
    setMotors(calPwmUsed, calPwmUsed);
  }
  motorsBrake();
  delay(400);
  motorsOff();
  if (aborted) Serial.println("[CAL] Dibatalkan: ada penghalang di depan (< 20 cm). Ulangi dengan lintasan lebih panjang.");
  else Serial.println("[CAL] Selesai. Ukur jarak tempuh (mm) sampai robot berhenti, lalu ketik: cald=<jarak mm>");
}

void speedCalibrateApply(float distMm) {
  if (calPwmUsed <= 0) {
    Serial.println("[CAL] jalankan perintah \"cal\" dulu");
    return;
  }
  if (distMm < 50.0f || distMm > 3000.0f) {
    Serial.println("[CAL] jarak tidak masuk akal");
    return;
  }
  float effS = (float)(CAL_RUN_MS - CAL_RAMP_MS) / 1000.0f;
  float v = distMm / effS;
  P.speedFullMmS = v * 255.0f / (float)calPwmUsed;
  Serial.printf("[CAL] v=%.0f mm/s pada PWM %d -> vf=%.0f (ketik save untuk menyimpan)\n", v, calPwmUsed, P.speedFullMmS);
}

// =====================================================================
// 13. PENYIMPANAN & SERIAL CLI (maintenance & tuning cepat)
// =====================================================================
void paramsLoad() {
  Params tmp;
  size_t n = prefs.getBytes("cfg", &tmp, sizeof(tmp));
  if (n == sizeof(Params) && tmp.ver == PARAMS_VER) {
    P = tmp;
    Serial.println("[CFG] dimuat dari flash");
  } else Serial.println("[CFG] memakai nilai default");
  sanitizeParams();
}
void paramsSave() {
  sanitizeParams();
  prefs.putBytes("cfg", &P, sizeof(P));
  Serial.println("[CFG] tersimpan");
}

void printConfig() {
  Serial.printf("kp=%.3f ki=%.3f kd=%.3f mc=%d\n", P.kp, P.ki, P.kd, P.maxCorr);
  Serial.printf("bs=%d ms=%d ts=%d mn=%d\n", P.baseSpeed, P.maxSpeed, P.turnSpeed, P.minSpeed);
  Serial.printf("tg=%d op=%d ft=%d fs=%d fc=%d\n", P.targetSide, P.openSide, P.frontTurn, P.frontSlow, P.frontClear);
  Serial.printf("sg=%u cy=%u vf=%.0f bd=%.0f lag=%u sa=%d\n", P.usGapUs, P.cycMs, P.speedFullMmS,
                P.brakeDecelMmS2, P.lagMs, P.sensorAxleMm);
  Serial.printf("arc=%d adv=%u rec=%u tto=%u stk=%u\n", P.arcPct, P.outerAdvanceMs, P.recoverMs, P.turnTimeout, P.stuckMs);
  Serial.printf("fin=%d fside=%d fhold=%u rto=%lu tel=%d\n", P.finishDetect, P.finishSide, P.finishHold,
                (unsigned long)P.runTimeout, P.telemetry);
  Serial.printf("side=%s cp=%d state=%s sensor=%s\n", trackRight ? "r" : "l", selCp, SYS_NAMES[sys],
                sensorsReady ? "OK" : "GAGAL");
  int baseS = P.baseSpeed * cpSpeedPct / 100;
  int sLow = min(P.minSpeed, baseS);
  Serial.printf("v@bs=%.0f mm/s  mulai melambat di F=%d mm  pivot90=%.0f ms  siklus=%.1f ms\n",
                speedMmS(baseS), dynFrontSlow(baseS, sLow),
                (speedMmS(P.turnSpeed) > 1.0f) ? (1.5708f / (2.0f * speedMmS(P.turnSpeed) / TRACK_WIDTH_MM)) * 1000.0f : 0.0f,
                loopMsEma);
  Serial.printf("percobaan=%u retry=%u/%u\n", runCount, retryCount, MAX_RETRY);
}

void printHelp() {
  Serial.println(F("== PERINTAH =="));
  Serial.println(F("show | save | load | reset | state | sn (alir sensor) | mt (tes motor) | rc (reset hitungan)"));
  Serial.println(F("go | stop | side=l/r | cp=0..3 | cal (kalibrasi) | cald=<mm>"));
  Serial.println(F("PID: kp= ki= kd= mc=   Kecepatan: bs= ms= ts= mn="));
  Serial.println(F("Jarak(mm): tg= op= ft= fs= fc=   Sensor: sg= (jeda us) cy= (siklus ms)"));
  Serial.println(F("Model gerak: vf= (mm/s @255) bd= (mm/s2) lag= (ms) sa= (mm)"));
  Serial.println(F("Manuver: arc= adv= rec= tto= stk=   Finish: fin=0/1 fside= fhold= rto=   Debug: tel=0/1"));
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  int eq = line.indexOf('=');
  String key = (eq < 0) ? line : line.substring(0, eq);
  String val = (eq < 0) ? "" : line.substring(eq + 1);
  float v = val.toFloat();

  if (key == "help") printHelp();
  else if (key == "show") printConfig();
  else if (key == "save") paramsSave();
  else if (key == "load") paramsLoad();
  else if (key == "reset") {
    P = Params();
    Serial.println("[CFG] default dipulihkan (belum disimpan)");
  } else if (key == "state") Serial.printf("sys=%s nav=%s\n", SYS_NAMES[sys], NAV_NAMES[nav]);
  else if (key == "sn") {
    streamSensors = !streamSensors;
    Serial.printf("stream sensor %s\n", streamSensors ? "ON" : "OFF");
  } else if (key == "mt") {
    if (sys != SYS_RUN) motorTest();
  } else if (key == "cal") {
    if (sys != SYS_RUN) speedCalibrateRun();
  } else if (key == "cald") speedCalibrateApply(v);
  else if (key == "rc") {
    runCount = 0;
    retryCount = 0;
    Serial.println("[CNT] hitungan percobaan direset");
  } else if (key == "go") {
    if (sys == SYS_IDLE || sys == SYS_READY) startRun();
  } else if (key == "stop") {
    if (sys == SYS_RUN) endRun("perintah serial", false);
    else motorsOff();
  } else if (key == "side") {
    trackRight = val.startsWith("r");
    applySelection();
  } else if (key == "cp") {
    selCp = constrain((int)v, 0, 3);
    applySelection();
  } else if (key == "kp") P.kp = v;
  else if (key == "ki") P.ki = v;
  else if (key == "kd") P.kd = v;
  else if (key == "mc") P.maxCorr = (int)v;
  else if (key == "bs") P.baseSpeed = constrain((int)v, 0, 255);
  else if (key == "ms") P.maxSpeed = constrain((int)v, 0, 255);
  else if (key == "ts") P.turnSpeed = constrain((int)v, 0, 255);
  else if (key == "mn") P.minSpeed = constrain((int)v, 0, 255);
  else if (key == "tg") P.targetSide = (int)v;
  else if (key == "op") P.openSide = (int)v;
  else if (key == "ft") P.frontTurn = (int)v;
  else if (key == "fs") P.frontSlow = (int)v;
  else if (key == "fc") P.frontClear = (int)v;
  else if (key == "sg") P.usGapUs = (uint16_t)constrain((int)v, 0, 20000);
  else if (key == "cy") P.cycMs = (uint16_t)constrain((int)v, 0, 1000);
  else if (key == "vf") P.speedFullMmS = v;
  else if (key == "bd") P.brakeDecelMmS2 = v;
  else if (key == "lag") P.lagMs = (uint16_t)constrain((int)v, 0, 1000);
  else if (key == "sa") P.sensorAxleMm = (int)v;
  else if (key == "arc") P.arcPct = constrain((int)v, 0, 100);
  else if (key == "adv") P.outerAdvanceMs = (uint16_t)v;
  else if (key == "rec") P.recoverMs = (uint16_t)v;
  else if (key == "tto") P.turnTimeout = (uint16_t)v;
  else if (key == "stk") P.stuckMs = (uint16_t)v;
  else if (key == "fin") P.finishDetect = (v != 0);
  else if (key == "fside") P.finishSide = (int)v;
  else if (key == "fhold") P.finishHold = (uint16_t)v;
  else if (key == "rto") P.runTimeout = (uint32_t)v;
  else if (key == "tel") P.telemetry = (v != 0);
  else {
    Serial.println("Perintah tidak dikenal. Ketik: help");
    return;
  }
  Serial.println("OK");
}

void cliPoll() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length()) {
        handleCommand(buf);
        buf = "";
      }
    } else if (buf.length() < 48) buf += c;
  }
}

// =====================================================================
// 14. SETUP & LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Wall Follower ESP32 (HC-SR04 + DRV8833) ===");

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);

  digitalWrite(PIN_LED_RED, HIGH);
  digitalWrite(PIN_LED_GREEN, HIGH);
  motorsInit();

  prefs.begin("wallf", false);
  paramsLoad();

  sensorsReady = sensorsInit();

  delay(400);  // LED boot menyala sebentar
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);

  bool bothHeld = (digitalRead(PIN_BTN_LEFT) == LOW) && (digitalRead(PIN_BTN_RIGHT) == LOW);

  if (!sensorsReady) {
    setSys(SYS_ERROR);
  } else if (bothHeld) {
    streamSensors = true;
    setSys(SYS_MAINT);
  } else {
    setSys(SYS_IDLE);
  }

  printHelp();
}

void loop() {
  btnUpdate(btnL);
  btnUpdate(btnR);
  cliPoll();

  switch (sys) {
    case SYS_IDLE: idleStep(); break;
    case SYS_READY: readyStep(); break;
    case SYS_RUN: runStep(); break;
    case SYS_FINISHED: finishedStep(); break;
    case SYS_ERROR: errorStep(); break;
    case SYS_MAINT: maintStep(); break;
  }
  ledsUpdate();
}
