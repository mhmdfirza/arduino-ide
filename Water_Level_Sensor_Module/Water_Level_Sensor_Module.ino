#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Definisikan Pin
const int WATER_SENSOR_PIN = A0;
const int BUZZER_PIN = 8;  // Pin Digital 8 untuk Buzzer

// AMBANG BATAS SENSITIVITAS:
// Jika nilai sensor turun di bawah angka ini, alarm akan berbunyi
const int WATER_THRESHOLD = 500;

void setup() {
  Serial.begin(9600);

  pinMode(WATER_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);  // Atur pin buzzer sebagai OUTPUT

  // Matikan buzzer di awal program
  digitalWrite(BUZZER_PIN, LOW);

  // Inisialisasi OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED Allocation Failed!"));
    for (;;)
      ;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  Serial.println("--- Monitoring Banjir + Alarm Buzzer Dimulai ---");
}

void loop() {
  // Baca nilai analog dari sensor api (0 - 1023)
  int rawWaterValue = analogRead(WATER_SENSOR_PIN);

  // Tampilkan data di Serial Monitor
  Serial.print("Water Sensor Value: ");
  Serial.print(rawWaterValue);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("SYSTEM STATUS:");
  display.println("---------------------");

  // Logika pengecekan kondisi Api
  if (rawWaterValue > WATER_THRESHOLD) {
    Serial.println(" -> [STATUS: WATER DETECTED! ALARM ON]");

    // Tampilan OLED saat ada api
    display.setTextSize(2);
    display.setCursor(0, 25);
    display.println("WATER");
    display.println("RAISED!");

    // --- AKTIVITAS ALARM (BUZZER KONDISI ADA API) ---
    // Membuat suara putus-putus (beep-beep) berkecepatan tinggi
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  } else {
    Serial.println(" -> [STATUS: SAFE]");

    // Tampilan OLED saat kondisi aman
    display.setTextSize(1);
    display.setCursor(0, 25);
    display.println("WATER LEVEL SAFE");

    display.setTextSize(2);
    display.setCursor(0, 42);
    display.println("[ SAFE ]");

    // --- PASTIKAN BUZZER MATI SAAT AMAN ---
    digitalWrite(BUZZER_PIN, LOW);
    delay(300);  // Jeda pembacaan normal sejenak
  }

  display.display();
}
