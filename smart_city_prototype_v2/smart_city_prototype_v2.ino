/*
  ============================================================
  PROTOTYPE SMART CITY SEDERHANA - IoT (REVISI v2)
  Microcontroller : Arduino Uno
  Tanpa koneksi internet/WiFi - sistem berjalan mandiri (standalone)
  ============================================================
  Fitur (tidak berubah dari versi sebelumnya):
  1. Monitor jumlah lalu lintas + palang pintu parkir -> HW-201 + Servo
  2. Monitor kualitas udara (proxy suhu/kelembapan)    -> DHT11
  3. Smart streetlamp                                  -> LDR + LED
  4. Early warning Banjir                              -> Water Level Sensor
  5. Early warning Kebakaran                           -> Flame Sensor IR
  6. Early warning Gempa                                -> SW-420 Vibration Sensor
  7. OLED sebagai "videotron wajah kota" (tampilan utama + alert)

  CATATAN KELISTRIKAN PENTING:
  - Servo TIDAK diberi daya dari Arduino. VCC servo diambil dari
    baterai Li-ion 18650 2S (7.4V) terpisah -> turunkan dulu ke 5-6V
    pakai buck converter sebelum masuk VCC servo (standar servo analog
    max 6V). GND servo, GND baterai, dan GND Arduino WAJIB disatukan
    (common ground), hanya pin SIGNAL servo yang terhubung ke Arduino.

  Library yang dibutuhkan (Library Manager):
  - Adafruit SSD1306, Adafruit GFX Library
  - DHT sensor library (Adafruit) + Adafruit Unified Sensor
  - Servo (bawaan Arduino IDE)
  ============================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <Servo.h>

// ======================= KONFIGURASI PIN =======================
#define PIN_HW201        2   // interrupt, hitung kendaraan + trigger palang
#define PIN_DHT          3
#define PIN_FLAME        4
#define PIN_SW420        5
#define PIN_LED_LAMPU    6
#define PIN_BUZZER       7
#define PIN_BUTTON_MUTE  8
#define PIN_SERVO_SIGNAL 9  // hanya sinyal; VCC servo dari baterai 18650 2S terpisah

#define PIN_LDR          A0
#define PIN_WATERLEVEL   A1

#define DHT_TYPE DHT11

// ======================= KONFIGURASI OLED =======================
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

DHT dht(PIN_DHT, DHT_TYPE);
Servo servoPalang;

// ======================= THRESHOLD (SESUAIKAN SAAT KALIBRASI) =======================
const int LDR_THRESHOLD_GELAP      = 400;
const int WATER_THRESHOLD_BANJIR   = 500;
const int GETARAN_COUNT_THRESHOLD  = 5;
const unsigned long GETARAN_WINDOW_MS = 3000;

// ======================= KONFIGURASI PALANG PARKIR (SERVO HALUS) =======================
const int SERVO_POS_TUTUP = 0;
const int SERVO_POS_BUKA  = 90;
const unsigned long SERVO_STEP_INTERVAL_MS = 15; // kecepatan gerak: makin besar makin lambat/halus
const int SERVO_STEP_SIZE = 1;                   // derajat per langkah
const unsigned long GATE_HOLD_OPEN_MS = 3000;    // lama palang tertahan terbuka

enum GateState { GATE_CLOSED, GATE_OPENING, GATE_OPEN_WAIT, GATE_CLOSING };
GateState gateState = GATE_CLOSED;
int posisiServoSekarang = SERVO_POS_TUTUP;
int posisiServoTarget   = SERVO_POS_TUTUP;
unsigned long lastServoStepTime = 0;
unsigned long gateOpenTimestamp = 0;

// ======================= VARIABEL GLOBAL LAINNYA =======================
volatile unsigned long jumlahKendaraan = 0;
volatile bool vehicleDetectedFlag = false;
volatile unsigned long lastInterruptTime = 0;
const unsigned long DEBOUNCE_INTERRUPT_MS = 80;

unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 1000;

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 500; // lebih cepat agar animasi "LIVE" terasa hidup

bool statusBanjir = false;
bool statusKebakaran = false;
bool statusGempa = false;
bool lampuMenyala = false;

int pulseGetaran = 0;
unsigned long getaranWindowStart = 0;

bool alarmDimute = false;
unsigned long muteStartTime = 0;
const unsigned long MUTE_DURATION_MS = 10000;

unsigned long lastBuzzerToggle = 0;
bool buzzerState = false;
const unsigned long BUZZER_INTERVAL = 300;

bool liveDotState = false;
unsigned long lastLiveBlink = 0;

bool alertInvertState = false;
unsigned long lastAlertBlink = 0;
const unsigned long ALERT_BLINK_INTERVAL = 400;

// ======================= SETUP =======================
void setup() {
  Serial.begin(9600);

  pinMode(PIN_HW201, INPUT_PULLUP);
  pinMode(PIN_FLAME, INPUT);
  pinMode(PIN_SW420, INPUT);
  pinMode(PIN_LED_LAMPU, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON_MUTE, INPUT_PULLUP);

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED_LAMPU, LOW);

  dht.begin();

  servoPalang.attach(PIN_SERVO_SIGNAL);
  servoPalang.write(SERVO_POS_TUTUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED gagal diinisialisasi!");
  }
  tampilkanSplashScreen();

  attachInterrupt(digitalPinToInterrupt(PIN_HW201), isrHitungKendaraan, FALLING);

  getaranWindowStart = millis();
  Serial.println("Sistem Smart City v2 siap (standalone, tanpa internet).");
}

// ======================= LOOP UTAMA =======================
void loop() {
  cekTombolMute();

  if (vehicleDetectedFlag) {
    vehicleDetectedFlag = false;
    bukaGateJikaTertutup();
  }

  if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = millis();
    bacaSensorBahaya();
    kontrolLampuOtomatis();
  }

  updateGatePalang();   // state machine + gerak servo halus, dipanggil tiap loop
  kelolaBuzzer();

  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = millis();
    bool adaBahaya = statusBanjir || statusKebakaran || statusGempa;
    if (adaBahaya) {
      tampilkanPeringatanBahaya();
    } else {
      tampilkanModeNormal();
    }
  }

  if (alarmDimute && (millis() - muteStartTime >= MUTE_DURATION_MS)) {
    alarmDimute = false;
  }
}

// ======================= ISR: HITUNG KENDARAAN =======================
void isrHitungKendaraan() {
  unsigned long waktuSekarang = millis();
  if (waktuSekarang - lastInterruptTime > DEBOUNCE_INTERRUPT_MS) {
    jumlahKendaraan++;
    vehicleDetectedFlag = true; // diproses di loop(), bukan di dalam ISR
    lastInterruptTime = waktuSekarang;
  }
}

// ======================= PEMBACAAN SENSOR BAHAYA =======================
void bacaSensorBahaya() {
  int flameRead = digitalRead(PIN_FLAME);
  statusKebakaran = (flameRead == LOW); // cek polaritas modul flame sensor masing-masing

  int waterRead = analogRead(PIN_WATERLEVEL);
  statusBanjir = (waterRead > WATER_THRESHOLD_BANJIR);

  int getaranRead = digitalRead(PIN_SW420);
  if (getaranRead == HIGH) pulseGetaran++;

  if (millis() - getaranWindowStart >= GETARAN_WINDOW_MS) {
    statusGempa = (pulseGetaran >= GETARAN_COUNT_THRESHOLD);
    pulseGetaran = 0;
    getaranWindowStart = millis();
  }

  Serial.print("Api:"); Serial.print(statusKebakaran);
  Serial.print(" Banjir:"); Serial.print(statusBanjir);
  Serial.print(" Gempa:"); Serial.print(statusGempa);
  Serial.print(" | Kendaraan:"); Serial.println(jumlahKendaraan);
}

// ======================= KONTROL LAMPU OTOMATIS =======================
void kontrolLampuOtomatis() {
  int nilaiLDR = analogRead(PIN_LDR);
  lampuMenyala = (nilaiLDR < LDR_THRESHOLD_GELAP);
  digitalWrite(PIN_LED_LAMPU, lampuMenyala ? HIGH : LOW);
}

// ======================= PALANG PARKIR: STATE MACHINE =======================
void bukaGateJikaTertutup() {
  if (gateState == GATE_CLOSED) {
    posisiServoTarget = SERVO_POS_BUKA;
    gateState = GATE_OPENING;
  }
}

void updateGatePalang() {
  stepServoHalus(); // gerakkan servo selangkah demi selangkah menuju target

  switch (gateState) {
    case GATE_OPENING:
      if (posisiServoSekarang == SERVO_POS_BUKA) {
        gateState = GATE_OPEN_WAIT;
        gateOpenTimestamp = millis();
      }
      break;

    case GATE_OPEN_WAIT:
      if (millis() - gateOpenTimestamp >= GATE_HOLD_OPEN_MS) {
        posisiServoTarget = SERVO_POS_TUTUP;
        gateState = GATE_CLOSING;
      }
      break;

    case GATE_CLOSING:
      if (posisiServoSekarang == SERVO_POS_TUTUP) {
        gateState = GATE_CLOSED;
      }
      break;

    case GATE_CLOSED:
    default:
      // diam, menunggu kendaraan berikutnya (vehicleDetectedFlag)
      break;
  }
}

// Pergerakan servo halus non-blocking: 1 derajat per interval, bukan loncat langsung
void stepServoHalus() {
  if (posisiServoSekarang == posisiServoTarget) return;
  if (millis() - lastServoStepTime < SERVO_STEP_INTERVAL_MS) return;

  lastServoStepTime = millis();
  if (posisiServoSekarang < posisiServoTarget) {
    posisiServoSekarang += SERVO_STEP_SIZE;
  } else {
    posisiServoSekarang -= SERVO_STEP_SIZE;
  }
  servoPalang.write(posisiServoSekarang);
}

// ======================= BUZZER (NON-BLOCKING) =======================
void kelolaBuzzer() {
  bool adaBahaya = statusBanjir || statusKebakaran || statusGempa;

  if (adaBahaya && !alarmDimute) {
    if (millis() - lastBuzzerToggle >= BUZZER_INTERVAL) {
      lastBuzzerToggle = millis();
      buzzerState = !buzzerState;
      digitalWrite(PIN_BUZZER, buzzerState ? HIGH : LOW);
    }
  } else {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerState = false;
  }
}

// ======================= TOMBOL MUTE =======================
void cekTombolMute() {
  static unsigned long lastPress = 0;
  if (digitalRead(PIN_BUTTON_MUTE) == LOW) {
    if (millis() - lastPress > 300) {
      lastPress = millis();
      alarmDimute = true;
      muteStartTime = millis();
      Serial.println("Alarm dibisukan sementara.");
    }
  }
}

// ======================= SPLASH SCREEN AWAL =======================
void tampilkanSplashScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.drawRoundRect(4, 4, 120, 56, 6, SSD1306_WHITE);
  display.setCursor(20, 18);
  display.setTextSize(2);
  display.println("SMART");
  display.setCursor(20, 38);
  display.println("CITY");
  display.display();
  delay(1500);
}

// ======================= OLED: MODE NORMAL ("VIDEOTRON" KOTA) =======================
void tampilkanModeNormal() {
  float suhu = dht.readTemperature();
  float kelembapan = dht.readHumidity();

  updateLiveBlink();

  display.clearDisplay();

  // --- Header bar ---
  display.fillRect(0, 0, SCREEN_WIDTH, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print("KOTA PINTAR");

  // indikator LIVE berkedip di pojok kanan atas
  if (liveDotState) {
    display.fillCircle(118, 5, 2, SSD1306_BLACK);
  }
  display.setCursor(96, 2);
  display.print("LIVE");

  display.setTextColor(SSD1306_WHITE);

  // --- Baris lalu lintas + palang ---
  drawIconMobil(2, 16);
  display.setCursor(16, 17);
  display.print(jumlahKendaraan);
  display.print(" kend.");

  display.setCursor(80, 17);
  display.print(statusGatePendek());

  display.drawLine(0, 27, SCREEN_WIDTH, 27, SSD1306_WHITE);

  // --- Baris suhu & kelembapan ---
  drawIconTermometer(2, 31);
  display.setCursor(16, 32);
  if (isnan(suhu)) {
    display.print("--.-C");
  } else {
    display.print(suhu, 1);
    display.print("C");
  }

  drawIconTetesAir(70, 31);
  display.setCursor(84, 32);
  if (isnan(kelembapan)) {
    display.print("--%");
  } else {
    display.print(kelembapan, 0);
    display.print("%");
  }

  display.drawLine(0, 43, SCREEN_WIDTH, 43, SSD1306_WHITE);

  // --- Baris status lampu kota ---
  drawIconLampu(2, 47, lampuMenyala);
  display.setCursor(16, 48);
  display.print("Lampu Kota: ");
  display.print(lampuMenyala ? "ON" : "OFF");

  // --- Footer status aman ---
  display.setCursor(2, 57);
  display.print("Kondisi kota: AMAN");

  display.display();
}

// ======================= OLED: MODE PERINGATAN BAHAYA =======================
void tampilkanPeringatanBahaya() {
  updateAlertBlink();

  display.clearDisplay();

  // header dengan efek flash (invert warna tiap interval)
  display.fillRect(0, 0, SCREEN_WIDTH, 13, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(8, 3);
  display.print(alertInvertState ? "!! AWAS BAHAYA !!" : "!!!!!!!!!!!!!!!!!");

  display.setTextColor(SSD1306_WHITE);
  display.drawTriangle(60, 16, 54, 25, 66, 25, SSD1306_WHITE);
  display.setCursor(63, 19);
  display.print("!");

  int y = 30;
  if (statusKebakaran) {
    drawIconApi(2, y);
    display.setCursor(16, y + 1);
    display.print("KEBAKARAN AKTIF");
    y += 11;
  }
  if (statusBanjir) {
    drawIconTetesAir(2, y);
    display.setCursor(16, y + 1);
    display.print("SIAGA BANJIR");
    y += 11;
  }
  if (statusGempa) {
    drawIconGempa(2, y);
    display.setCursor(16, y + 1);
    display.print("GETARAN/GEMPA");
    y += 11;
  }

  display.drawLine(0, 56, SCREEN_WIDTH, 56, SSD1306_WHITE);
  display.setCursor(2, 57);
  display.print(alarmDimute ? "Alarm: BISU" : "Alarm: BUNYI");

  display.display();
}

// ======================= HELPER: STATUS GATE SINGKAT =======================
String statusGatePendek() {
  switch (gateState) {
    case GATE_CLOSED:    return "Tutup";
    case GATE_OPENING:   return "Buka..";
    case GATE_OPEN_WAIT: return "Buka";
    case GATE_CLOSING:   return "Tutup..";
    default:             return "-";
  }
}

// ======================= ANIMASI: LIVE BLINK (MODE NORMAL) =======================
void updateLiveBlink() {
  if (millis() - lastLiveBlink >= 500) {
    lastLiveBlink = millis();
    liveDotState = !liveDotState;
  }
}

// ======================= ANIMASI: ALERT BLINK (MODE BAHAYA) =======================
void updateAlertBlink() {
  if (millis() - lastAlertBlink >= ALERT_BLINK_INTERVAL) {
    lastAlertBlink = millis();
    alertInvertState = !alertInvertState;
  }
}

// ======================= IKON-IKON SEDERHANA (GFX PRIMITIVES) =======================
// Dibuat dengan shape dasar agar ringan & mudah dimodifikasi/ditambah developer lain.

void drawIconMobil(int x, int y) {
  display.drawRoundRect(x, y + 2, 12, 6, 2, SSD1306_WHITE);
  display.fillCircle(x + 3, y + 8, 1, SSD1306_WHITE);
  display.fillCircle(x + 9, y + 8, 1, SSD1306_WHITE);
}

void drawIconTermometer(int x, int y) {
  display.drawLine(x + 2, y, x + 2, y + 8, SSD1306_WHITE);
  display.drawCircle(x + 2, y + 10, 2, SSD1306_WHITE);
}

void drawIconTetesAir(int x, int y) {
  display.drawTriangle(x + 2, y, x, y + 5, x + 4, y + 5, SSD1306_WHITE);
  display.drawCircle(x + 2, y + 6, 2, SSD1306_WHITE);
}

void drawIconLampu(int x, int y, bool nyala) {
  if (nyala) {
    display.fillCircle(x + 3, y + 3, 4, SSD1306_WHITE);
  } else {
    display.drawCircle(x + 3, y + 3, 4, SSD1306_WHITE);
  }
  display.drawLine(x + 1, y + 8, x + 5, y + 8, SSD1306_WHITE);
}

void drawIconApi(int x, int y) {
  display.drawTriangle(x + 4, y, x, y + 8, x + 8, y + 8, SSD1306_WHITE);
  display.drawLine(x + 4, y + 3, x + 4, y + 8, SSD1306_WHITE);
}

void drawIconGempa(int x, int y) {
  display.drawLine(x, y + 4, x + 2, y, SSD1306_WHITE);
  display.drawLine(x + 2, y, x + 5, y + 8, SSD1306_WHITE);
  display.drawLine(x + 5, y + 8, x + 8, y, SSD1306_WHITE);
}
