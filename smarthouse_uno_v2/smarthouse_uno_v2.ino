/* ======================================================================
   SMART CITY IoT - ARDUINO UNO (STANDALONE, TANPA INTERNET/WIFI)  v2
   ----------------------------------------------------------------------
   PENYESUAIAN DARI VERSI SEBELUMNYA:
     1. Tanpa ESP8266/koneksi apa pun - 100% berjalan mandiri di Arduino.
        (file smarthouse_esp8266.ino TIDAK dipakai lagi)
     2. OLED jadi "videotron wajah kota": ikon status + siluet skyline +
        jam uptime, dan layar alert penuh untuk banjir/kebakaran.
     3. Servo bergerak HALUS & non-blocking (tidak jerky), palang parkir
        dibuat lebih pelan agar menyerupai palang sungguhan.
     4. Smart Lamp disederhanakan: LED hanya indikator lampu (on/off),
        karena tugas alarm kini sepenuhnya di OLED + buzzer.

   >>> CATATAN HARDWARE WAJIB DIBACA (rel daya servo) <<<
   Servo diberi daya dari baterai Li-ion 18650 2S (7.4V) TERPISAH dari
   Arduino (tidak numpang pin 5V Arduino). SG90 hanya aman di 4.8-6V,
   maka WAJIB pasang modul buck converter kecil (mis. MP1584EN) untuk
   menurunkan 7.4V -> ~5V sebelum masuk VCC servo. GND baterai servo
   dan GND Arduino tetap WAJIB disatukan (common ground).

   Fitur: Jemuran Otomatis, Smart Lamp, Smart Door Lock (password),
          Flood Detector, Fire Outbreak Detector, Palang Parkir Otomatis
   Library: Servo, Wire, Adafruit GFX, Adafruit SSD1306
   ====================================================================== */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdio.h>

#define DEBUG 0   // set ke 1 untuk lihat nilai sensor mentah di Serial Monitor

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* -------------------- PETA PIN -------------------- */
#define PIN_LDR        A0
#define PIN_RAIN       A1
#define PIN_WATER      A2
// A3, D0, D1 = cadangan (bekas jalur ESP8266, kini tidak dipakai)

#define PIN_IR_PARKIR     2
#define PIN_FLAME         3
#define PIN_SERVO_JEMURAN 4
#define PIN_RGB_R         5 
#define PIN_RGB_G         6
#define PIN_BTN1          7
#define PIN_BTN2          8
#define PIN_SERVO_PARKIR  9
#define PIN_SERVO_LOCK    10
#define PIN_RGB_B         11
#define PIN_BTN3          12
#define PIN_BUZZER        13

/* ======================================================================
   SERVO HALUS - util reusable, non-blocking smooth move
   Pola ini scalable: tinggal tambah 1 baris struct untuk servo baru.
   ====================================================================== */
struct ServoHalus {
  Servo servo;
  int sudutSekarang;
  int sudutTarget;
  unsigned long waktuLangkah;
  int msPerDerajat;   // makin besar = makin pelan & lembut
};

ServoHalus sJemuran = {Servo(), 90, 90, 0, 10};
ServoHalus sParkir  = {Servo(),  0,  0, 0, 18};  // sengaja lebih pelan -> kesan palang asli
ServoHalus sLock    = {Servo(),  0,  0, 0, 10};

void aturTargetServo(ServoHalus &s, int target) {
  s.sudutTarget = constrain(target, 0, 180);
}

void perbaruiServoHalus(ServoHalus &s, unsigned long now) {
  if (s.sudutSekarang == s.sudutTarget) return;
  if (now - s.waktuLangkah >= (unsigned long)s.msPerDerajat) {
    s.waktuLangkah = now;
    s.sudutSekarang += (s.sudutSekarang < s.sudutTarget) ? 1 : -1;
    s.servo.write(s.sudutSekarang);
  }
}

/* -------------------- KALIBRASI (sesuaikan dgn kondisi nyata) -------------------- */
int thresholdLDR   = 500;
int thresholdRain  = 500;
int thresholdWater = 300;

/* -------------------- STATUS SISTEM -------------------- */
bool lampuOn         = false;
bool jemuranTertutup = false;
bool palangTerbuka   = false;
bool pintuTerkunci   = true;
bool banjirAktif     = false;
bool apiAktif        = false;

/* -------------------- TIMER NON-BLOCKING -------------------- */
unsigned long tParkir = 0;
const unsigned long DURASI_PALANG_TERBUKA = 5000;

unsigned long tLockFeedback = 0;
bool lockFeedbackAktif = false;
bool lockFeedbackHasil = false;
const unsigned long DURASI_FEEDBACK = 1500;

unsigned long tAutoLock = 0;
const unsigned long DURASI_NUTUP = 3000;

unsigned long tSensorRead = 0;
const unsigned long INTERVAL_SENSOR = 200;

unsigned long tOled = 0;
const unsigned long INTERVAL_OLED = 250;

unsigned long tDebounceTombol = 0;
unsigned long tBoot = 0;   // acuan jam uptime di videotron

/* -------------------- PASSWORD -------------------- */
int passwordBenar[4] = {1, 2, 1, 3};
int inputPassword[4];
int jumlahInput = 0;

/* ====================================================================== */
void setup() {
#if DEBUG
  Serial.begin(9600);
#endif

  pinMode(PIN_IR_PARKIR, INPUT);
  pinMode(PIN_FLAME, INPUT);
  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  pinMode(PIN_BTN3, INPUT_PULLUP);
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  sJemuran.servo.attach(PIN_SERVO_JEMURAN);
  sParkir.servo.attach(PIN_SERVO_PARKIR);
  sLock.servo.attach(PIN_SERVO_LOCK);
  sJemuran.servo.write(sJemuran.sudutSekarang);
  sParkir.servo.write(sParkir.sudutSekarang);
  sLock.servo.write(sLock.sudutSekarang);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // sebagian modul pakai 0x3D
  tBoot = millis();
  layarSplash();   // hanya sekali saat boot, tidak mengganggu loop utama
}

/* ====================================================================== */
void loop() {
  unsigned long now = millis();

  bacaTombolPassword(now);

  if (now - tSensorRead >= INTERVAL_SENSOR) {
    tSensorRead = now;
    cekApi();
    cekBanjir();
    cekLampu();
    cekJemuran();
  }

  cekPalangParkir(now);
  cekAutoLock(now);
  updateLockFeedback(now);
  updateLampu();
  updateBuzzerAlarm(now);

  perbaruiServoHalus(sJemuran, now);
  perbaruiServoHalus(sParkir, now);
  perbaruiServoHalus(sLock, now);

  if (now - tOled >= INTERVAL_OLED) {
    tOled = now;
    tampilkanOLED(now);
  }
}

/* ============== FITUR 1: SMART LAMP (LDR + LED) ============== */
void cekLampu() {
  int nilai = analogRead(PIN_LDR);
  lampuOn = (nilai > thresholdLDR);
#if DEBUG
  Serial.print("LDR:"); Serial.println(nilai);
#endif
}

void updateLampu() {
  if (lampuOn) { analogWrite(PIN_RGB_R, 255); analogWrite(PIN_RGB_G, 170); analogWrite(PIN_RGB_B, 80); }
  else         { analogWrite(PIN_RGB_R, 0);   analogWrite(PIN_RGB_G, 0);   analogWrite(PIN_RGB_B, 0);  }
}

/* ============== FITUR 2: JEMURAN OTOMATIS (MH-RD Rain + Servo) ============== */
void cekJemuran() {
  int nilai = analogRead(PIN_RAIN);
  bool hujan = (nilai > thresholdRain);
  if (hujan != jemuranTertutup) {
    jemuranTertutup = hujan;
    aturTargetServo(sJemuran, jemuranTertutup ? 0 : 90);
  }
#if DEBUG
  Serial.print("RAIN:"); Serial.println(nilai);
#endif
}

/* ============== FITUR 3: PALANG PARKIR OTOMATIS (IR + Servo halus) ============== */
void cekPalangParkir(unsigned long now) {
  bool adaMobil = (digitalRead(PIN_IR_PARKIR) == HIGH);
  if (adaMobil && !palangTerbuka) {
    palangTerbuka = true;
    aturTargetServo(sParkir, 90);
    tParkir = now;
  }
  if (palangTerbuka && (now - tParkir >= DURASI_PALANG_TERBUKA)) {
    palangTerbuka = false;
    aturTargetServo(sParkir, 0);
  }
}

/* ============== FITUR 4: SMART DOOR LOCK (Password 3 Tombol) ============== */
void bacaTombolPassword(unsigned long now) {
  if (now - tDebounceTombol < 200) return;

  int tombol = 0;
  if (digitalRead(PIN_BTN1) == LOW) tombol = 1;
  else if (digitalRead(PIN_BTN2) == LOW) tombol = 2;
  else if (digitalRead(PIN_BTN3) == LOW) tombol = 3;

  if (tombol != 0) {
    tDebounceTombol = now;
    if (jumlahInput < 4) inputPassword[jumlahInput++] = tombol;
    if (jumlahInput == 4) prosesPassword(now);
  }
}

void prosesPassword(unsigned long now) {
  bool cocok = true;
  for (int i = 0; i < 4; i++) if (inputPassword[i] != passwordBenar[i]) cocok = false;

  lockFeedbackAktif = true;
  lockFeedbackHasil = cocok;
  tLockFeedback = now;

  if (cocok) {
    pintuTerkunci = false;
    aturTargetServo(sLock, 90);
    tAutoLock = now;
  } else {
    tone(PIN_BUZZER, 1000, 300);
  }
  jumlahInput = 0;
}

void cekAutoLock(unsigned long now) {
  if (!pintuTerkunci && (now - tAutoLock >= DURASI_NUTUP)) {
    pintuTerkunci = true;
    aturTargetServo(sLock, 0);
  }
}

void updateLockFeedback(unsigned long now) {
  if (lockFeedbackAktif && (now - tLockFeedback >= DURASI_FEEDBACK)) {
    lockFeedbackAktif = false;
  }
}

/* ============== FITUR 5: FLOOD DETECTOR (Water Level + OLED + Buzzer) ============== */
void cekBanjir() {
  int nilai = analogRead(PIN_WATER);
  banjirAktif = (nilai > thresholdWater);
#if DEBUG
  Serial.print("WATER:"); Serial.println(nilai);
#endif
}

/* ============== FITUR 6: FIRE OUTBREAK DETECTOR (Flame + OLED + Buzzer) ============== */
void cekApi() {
  apiAktif = (digitalRead(PIN_FLAME) == LOW);
}

/* ============== BUZZER: pola alarm (prioritas api > banjir) ============== */
void updateBuzzerAlarm(unsigned long now) {
  if (apiAktif) {
    digitalWrite(PIN_BUZZER, ((now / 150) % 2 == 0) ? HIGH : LOW);
    return;
  }
  if (banjirAktif) {
    digitalWrite(PIN_BUZZER, ((now / 400) % 2 == 0) ? HIGH : LOW);
    return;
  }
  if (lockFeedbackAktif) return;   // jangan ganggu tone() password yang sedang berjalan
  noTone(PIN_BUZZER);
  digitalWrite(PIN_BUZZER, LOW);
}

/* ======================================================================
   OLED "VIDEOTRON WAJAH KOTA" - tampilan status & alert
   ====================================================================== */
void layarSplash() {
  display.clearDisplay();
  display.drawRoundRect(2, 2, 124, 60, 6, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(22, 18); display.print("SMART CITY");
  display.setCursor(34, 30); display.print("MONITOR");
  display.setCursor(10, 46); display.print("Menyalakan kota...");
  display.display();
  delay(800);
}

void gambarSkyline() {
  static const int gedung[10][2] = {
    {2,8},{14,14},{28,6},{40,16},{54,10},{68,18},{82,8},{96,14},{110,6},{120,12}
  };
  for (int i = 0; i < 10; i++) {
    display.fillRect(gedung[i][0], 64 - gedung[i][1], 10, gedung[i][1], SSD1306_WHITE);
  }
}

void ikonLampu(int x, int y, bool on) {
  display.drawCircle(x, y, 5, SSD1306_WHITE);
  if (on) {
    display.fillCircle(x, y, 3, SSD1306_WHITE);
    display.drawLine(x - 8, y, x - 6, y, SSD1306_WHITE);
    display.drawLine(x + 6, y, x + 8, y, SSD1306_WHITE);
    display.drawLine(x, y - 8, x, y - 6, SSD1306_WHITE);
  }
}

void ikonJemuran(int x, int y, bool tertutup) {
  display.drawLine(x - 7, y, x + 7, y, SSD1306_WHITE);
  if (!tertutup) {
    display.drawLine(x - 4, y, x - 4, y + 5, SSD1306_WHITE);
    display.drawLine(x + 4, y, x + 4, y + 5, SSD1306_WHITE);
  }
}

void ikonPalang(int x, int y, bool terbuka) {
  display.drawLine(x - 7, y + 6, x + 7, y + 6, SSD1306_WHITE);
  if (terbuka) display.drawLine(x - 6, y + 6, x + 2, y - 5, SSD1306_WHITE);
  else         display.drawLine(x - 6, y + 6, x + 7, y + 6, SSD1306_WHITE);
}

void ikonGembok(int x, int y, bool terkunci) {
  display.drawRoundRect(x - 5, y, 10, 7, 2, SSD1306_WHITE);
  if (terkunci) display.drawRect(x - 3, y - 5, 6, 6, SSD1306_WHITE);
  else          display.drawLine(x - 3, y - 5, x - 3, y - 1, SSD1306_WHITE);
}

void tampilanNormal(unsigned long now) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0); display.print("KOTA PINTAR");
  unsigned long detik = (now - tBoot) / 1000;
  char jam[9];
  sprintf(jam, "%02lu:%02lu:%02lu", (detik / 3600) % 24, (detik / 60) % 60, detik % 60);
  display.setCursor(74, 0); display.print(jam);
  display.drawFastHLine(0, 9, 128, SSD1306_WHITE);

  ikonLampu(16, 24, lampuOn);
  ikonJemuran(48, 24, jemuranTertutup);
  ikonPalang(80, 24, palangTerbuka);
  ikonGembok(112, 24, pintuTerkunci);

  display.setCursor(1, 34);  display.print("Lampu");
  display.setCursor(33, 34); display.print("Jemur");
  display.setCursor(65, 34); display.print("Parkr");
  display.setCursor(97, 34); display.print("Pintu");

  display.drawFastHLine(0, 44, 128, SSD1306_WHITE);
  gambarSkyline();

  if (lockFeedbackAktif) {
    display.fillRect(14, 46, 100, 14, SSD1306_BLACK);
    display.drawRect(14, 46, 100, 14, SSD1306_WHITE);
    display.setCursor(20, 49);
    display.print(lockFeedbackHasil ? "AKSES DITERIMA" : "AKSES DITOLAK");
  }
  display.display();
}

void tampilanAlertBanjir(unsigned long now) {
  bool kedip = ((now / 500) % 2 == 0);
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  if (kedip) display.fillRect(2, 2, 124, 60, SSD1306_WHITE);
  display.setTextColor(kedip ? SSD1306_BLACK : SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(10, 14); display.print("BANJIR!");
  display.setTextSize(1);
  display.setCursor(6, 40);  display.print("Genangan air terdeteksi");
  display.display();
}

void tampilanAlertApi(unsigned long now) {
  bool kedip = ((now / 180) % 2 == 0);
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  if (kedip) display.fillRect(2, 2, 124, 60, SSD1306_WHITE);
  display.setTextColor(kedip ? SSD1306_BLACK : SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(2, 14);  display.print("KEBAKARAN");
  display.setTextSize(1);
  display.setCursor(8, 40);  display.print("Titik api terdeteksi!");
  display.setCursor(8, 50);  display.print("Evakuasi & cek sumber");
  display.display();
}

void tampilkanOLED(unsigned long now) {
  if (apiAktif)          tampilanAlertApi(now);
  else if (banjirAktif)  tampilanAlertBanjir(now);
  else                   tampilanNormal(now);
}
