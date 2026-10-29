#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// Konfigurasi OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Inisialisasi Pin
const int pinSensor = A0;  // Pin Analog sensor air MHRD
const int pinBuzzer = 8;   // Pin Digital untuk Buzzer
const int pinServo = 9;    // Pin Digital untuk Servo

// Objek Servo
Servo servoJemuran;

// Batas nilai sensor (Threshold)
const int batasHujan = 700; 

// Pengaturan Nada Buzzer
const int frekuensiNada = 1500; // Frekuensi nada dalam Hertz (makin besar makin melengking)

void setup() {
  // Inisialisasi Serial Monitor
  Serial.begin(9600);
  
  // Inisialisasi Pin
  pinMode(pinSensor, INPUT);
  pinMode(pinBuzzer, OUTPUT);
  
  // Inisialisasi Servo
  servoJemuran.attach(pinServo);
  servoJemuran.write(0); // Posisi awal jemuran (terbuka / aman)

  // Inisialisasi OLED Display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("OLED gagal diinisialisasi!"));
    for(;;); 
  }
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println("Sistem Siap...");
  display.display();
  delay(2000);
}

void loop() {
  // Membaca nilai dari sensor air
  int nilaiSensor = analogRead(pinSensor);
  
  // Cetak ke Serial Monitor untuk keperluan debugging
  Serial.print("Nilai Sensor: ");
  Serial.println(nilaiSensor);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Sensor: ");
  display.println(nilaiSensor);

  // Logika pengecekan hujan
  if (nilaiSensor < batasHujan) {
    // KONDISI HUJAN
    servoJemuran.write(90); // Menggerakkan servo ke 90 derajat (menarik jemuran)
    
    // perbaikan: Bunyikan alarm menggunakan fungsi tone() agar mendukung buzzer pasif & aktif
    tone(pinBuzzer, frekuensiNada); 
    delay(200);                     // Bunyi selama 0.2 detik
    noTone(pinBuzzer);              // Jeda mati sebentar (efek putus-putus)
    
    // Tampilkan Status di OLED
    display.setTextSize(2);
    display.setCursor(0, 30);
    display.println("STATUS: HUJAN");
    display.setTextSize(1);
    display.println("Jemuran Ditarik!");
    
  } else {
    // KONDISI AMAN / KERING
    servoJemuran.write(0);         // Mengembalikan servo ke posisi awal (0 derajat)
    
    // perbaikan: Pastikan buzzer benar-benar mati saat kering
    noTone(pinBuzzer); 
    
    // Tampilkan Status di OLED
    display.setTextSize(2);
    display.setCursor(0, 30);
    display.println("STATUS: AMAN");
    display.setTextSize(1);
    display.println("Jemuran Aman");
  }

  display.display();
  delay(300); // Delay disesuaikan menjadi 0.3 detik agar respons sensor dan alarm lebih cepat
}
