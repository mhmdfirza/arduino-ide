/* ======================================================================
SMART HOUSE IoT SEDERHANA - ARDUINO UNO (KODE UTAMA)
Fitur : Jemuran Otomatis, Smart Lamp, Palang Parkir Otomatis,
Smart Door Lock (password), Deteksi Banjir, Deteksi Kebakaran
Papan : Arduino Uno
Library yang harus diinstall via Library Manager:
- Servo
- Adafruit GFX Library
- Adafruit SSD1306
====================================================================== */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* -------------------- PETA PIN -------------------- */
#define PIN_LDR A0
#define PIN_RAIN A1
#define PIN_WATER A2
// A3 = cadangan/spare
#define PIN_IR_PARKIR 2
#define PIN_FLAME 3
#define PIN_SERVO_JEMURAN 4
#define PIN_RGB_R 5
#define PIN_RGB_G 6
#define PIN_BTN1 7
#define PIN_BTN2 8
#define PIN_SERVO_PARKIR 9
#define PIN_SERVO_LOCK 10
#define PIN_RGB_B 11
#define PIN_BTN3 12
#define PIN_BUZZER 13
Servo servoJemuran, servoParkir, servoLock;

/* -------------------- KALIBRASI (sesuaikan dengan kondisi nyata) -------------------- */
int thresholdLDR = 500; // < nilai ini dianggap gelap -> lampu nyala
int thresholdRain = 500; // < nilai ini dianggap hujan (cek nilai asli via Serial Monitor)
int thresholdWater = 300; // > nilai ini dianggap ada air/banjir

/* -------------------- STATUS SISTEM -------------------- */
bool lampuOn = false;
bool jemuranTertutup = false;
bool palangTerbuka = false;
bool pintuTerkunci = true;
bool banjirAktif = false;
bool apiAktif = false;

/* -------------------- TIMER NON-BLOCKING -------------------- */
unsigned long tParkir = 0;
const unsigned long DURASI_PALANG_TERBUKA = 5000; // 5 detik
unsigned long tLockFeedback = 0;
bool lockFeedbackAktif = false;
bool lockFeedbackHasil = false;
const unsigned long DURASI_FEEDBACK = 2000; // 2 detik
unsigned long tAutoLock = 0;
const unsigned long DURASI_AUTO_LOCK = 8000; // pintu otomatis terkunci lagi setelah 8 detik
unsigned long tSensorRead = 0;
const unsigned long INTERVAL_SENSOR = 200;
unsigned long tKirimStatus = 0;
const unsigned long INTERVAL_KIRIM = 1000; // kirim status ke ESP8266 tiap 1 detik
unsigned long tOled = 0;
const unsigned long INTERVAL_OLED = 300;
unsigned long tDebounceTombol = 0;

/* -------------------- PASSWORD -------------------- */
// Password default: Tombol1, Tombol2, Tombol1, Tombol3 -> silakan ganti sesuai keinginan
int passwordBenar[4] = {1, 2, 1, 3};
int inputPassword[4];
int jumlahInput = 0;

/*  SETUP ====================================================================== */
void setup() {
  Serial.begin(9600); // Serial ini dipakai juga untuk mengirim data ke ESP8266 (D1/TX)
  pinMode(PIN_IR_PARKIR, INPUT);
  pinMode(PIN_FLAME, INPUT);
  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  pinMode(PIN_BTN3, INPUT_PULLUP);
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  servoJemuran.attach(PIN_SERVO_JEMURAN);
  servoParkir.attach(PIN_SERVO_PARKIR);
  servoLock.attach(PIN_SERVO_LOCK);
  servoJemuran.write(90); // posisi awal: jemuran terbuka (cerah)
  servoParkir.write(0); // posisi awal: palang tertutup
  servoLock.write(0); // posisi awal: pintu terkunci
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // sebagian modul memakai alamat 0x3D
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("SMART HOUSE IoT");
  display.println("Sistem siap...");
  display.display();
  delay(1000);
}


/* LOOP ====================================================================== */
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
updateAlarmVisual(now);
if (now - tOled >= INTERVAL_OLED) {
tOled = now;
tampilkanOLED();
}
if (now - tKirimStatus >= INTERVAL_KIRIM) {
tKirimStatus = now;
kirimStatusKeESP();
}
}
/* ============== FITUR 1: SMART LAMP (LDR) ============== */
void cekLampu() {
int nilaiLDR = analogRead(PIN_LDR);
lampuOn = (nilaiLDR < thresholdLDR);
}
/* ============== FITUR 2: JEMURAN OTOMATIS (Rain Sensor) ============== */
void cekJemuran() {
int nilaiHujan = analogRead(PIN_RAIN);
bool hujanSekarang = (nilaiHujan < thresholdRain);
if (hujanSekarang != jemuranTertutup) {
jemuranTertutup = hujanSekarang;
servoJemuran.write(jemuranTertutup ? 0 : 90);
}
}
/* ============== FITUR 3: PALANG PARKIR OTOMATIS (IR Sensor) ============== */
void cekPalangParkir(unsigned long now) {
bool adaMobil = (digitalRead(PIN_IR_PARKIR) == LOW); // aktif LOW saat ada objek
if (adaMobil && !palangTerbuka) {
palangTerbuka = true;
servoParkir.write(90);
tParkir = now;
}
if (palangTerbuka && (now - tParkir >= DURASI_PALANG_TERBUKA)) {
palangTerbuka = false;
servoParkir.write(0);
}
}
/* ============== FITUR 4: SMART DOOR LOCK (Password 3 Tombol) ============== */
void bacaTombolPassword(unsigned long now) {
if (now - tDebounceTombol < 200) return; // debounce sederhana
int tombol = 0;
if (digitalRead(PIN_BTN1) == LOW) tombol = 1;
else if (digitalRead(PIN_BTN2) == LOW) tombol = 2;
else if (digitalRead(PIN_BTN3) == LOW) tombol = 3;
if (tombol != 0) {
tDebounceTombol = now;
if (jumlahInput < 4) {
inputPassword[jumlahInput] = tombol;
jumlahInput++;
}
if (jumlahInput == 4) {
prosesPassword(now);
}
}
}
void prosesPassword(unsigned long now) {
bool cocok = true;
for (int i = 0; i < 4; i++) {
if (inputPassword[i] != passwordBenar[i]) cocok = false;
}
lockFeedbackAktif = true;
lockFeedbackHasil = cocok;
tLockFeedback = now;
if (cocok) {
pintuTerkunci = false;
servoLock.write(90);
tAutoLock = now;
} else {
  tone(PIN_BUZZER, 1000, 300);
}
jumlahInput = 0;
}
void cekAutoLock(unsigned long now) {
if (!pintuTerkunci && (now - tAutoLock >= DURASI_AUTO_LOCK)) {
pintuTerkunci = true;
servoLock.write(0);
}
}
/* ============== FITUR 5: DETEKSI BANJIR (Water Sensor) ============== */
void cekBanjir() {
int nilaiAir = analogRead(PIN_WATER);
banjirAktif = (nilaiAir > thresholdWater);
}
/* ============== FITUR 6: DETEKSI KEBAKARAN (Flame Sensor) ============== */
void cekApi() {
apiAktif = (digitalRead(PIN_FLAME) == LOW); // aktif LOW saat api terdeteksi
}
/* ============== RGB LED + BUZZER: PRIORITAS ALARM ============== */
void updateAlarmVisual(unsigned long now) {
if (apiAktif) {
// prioritas tertinggi: merah berkedip cepat + buzzer menyala
bool kedip = ((now / 150) % 2 == 0);
setRGB(kedip ? 255 : 0, 0, 0);
digitalWrite(PIN_BUZZER, kedip ? HIGH : LOW);
}
else if (banjirAktif) {
// biru berkedip + buzzer beep-beep
bool kedip = ((now / 400) % 2 == 0);
setRGB(0, 0, kedip ? 255 : 0);
digitalWrite(PIN_BUZZER, kedip ? HIGH : LOW);
}
else if (lockFeedbackAktif) {
if (now - tLockFeedback < DURASI_FEEDBACK) {
if (lockFeedbackHasil) setRGB(0, 255, 0); // hijau = akses diterima
else setRGB(255, 0, 0); // merah = akses ditolak
} else {
lockFeedbackAktif = false;
}
}
else {
noTone(PIN_BUZZER);
digitalWrite(PIN_BUZZER, LOW);
if (lampuOn) setRGB(255, 170, 80); // warna hangat, lampu menyala
else setRGB(0, 0, 0); // lampu mati
}
}
void setRGB(int r, int g, int b) {
analogWrite(PIN_RGB_R, r);
analogWrite(PIN_RGB_G, g);
analogWrite(PIN_RGB_B, b);
}
/* ============== TAMPILAN OLED ============== */
void tampilkanOLED() {
display.clearDisplay();
display.setCursor(0, 0);
display.setTextSize(1);
if (apiAktif) {
display.println("!! BAHAYA !!");
display.println("API TERDETEKSI!");
} else if (banjirAktif) {
display.println("** PERINGATAN **");
display.println("BANJIR TERDETEKSI");
} else {
display.println("SMART HOUSE IoT");
display.print("Lampu : "); display.println(lampuOn ? "ON" : "OFF");
display.print("Jemuran : "); display.println(jemuranTertutup ? "TUTUP" : "BUKA");
display.print("Parkir : "); display.println(palangTerbuka ? "TERBUKA" : "TERTUTUP");
display.print("Pintu : "); display.println(pintuTerkunci ? "TERKUNCI" : "TERBUKA");
if (jumlahInput > 0) {
display.print("Pass: ");
for (int i = 0; i < jumlahInput; i++) display.print("*");
}
}
display.display();
}
/* ============== KIRIM STATUS KE ESP8266 (Serial, 1 arah) ============== */
void kirimStatusKeESP() {
Serial.print("LAMP:"); Serial.print(lampuOn ? 1 : 0);
Serial.print(",CLOTH:"); Serial.print(jemuranTertutup ? 1 : 0);
Serial.print(",GATE:"); Serial.print(palangTerbuka ? 1 : 0);
Serial.print(",LOCK:"); Serial.print(pintuTerkunci ? 1 : 0);
Serial.print(",FLOOD:"); Serial.print(banjirAktif ? 1 : 0);
Serial.print(",FIRE:"); Serial.println(apiAktif ? 1 : 0);
}
