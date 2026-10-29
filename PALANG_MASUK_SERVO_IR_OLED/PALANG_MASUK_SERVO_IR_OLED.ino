#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// Konfigurasi Layar OLED
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Konfigurasi Pin Komponen
const int pinIR = 2;     
const int pinServo = 9;  

// Inisialisasi Objek & Variabel
Servo palangServo;
int jumlahKendaraan = 0;
bool statusKendaraan = false;

// Variabel untuk Teks Berjalan
String pesanTeks = "Selamat datang di kota kami ";
int posisiX;

void setup() {
  Serial.begin(9600);
  
  pinMode(pinIR, INPUT);
  
  palangServo.attach(pinServo);
  palangServo.write(0); // Posisi awal palang menutup (0 derajat)
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED SSD1306 tidak terdeteksi!"));
    for(;;); 
  }
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  posisiX = SCREEN_WIDTH; 
  updateTampilanOLED();
}

void loop() {
  int irMembaca = digitalRead(pinIR);
  
  if (irMembaca == LOW && !statusKendaraan) {
    jumlahKendaraan++;
    statusKendaraan = true;
    
    updateTampilanOLED();
    
    // --- 1. MEMBUKA PALANG DENGAN LAMBAT (0 ke 90 derajat) ---
    // Menggunakan perulangan agar naik 1 derajat demi 1 derajat
    for (int sudut = 0; sudut <= 90; sudut++) {
      palangServo.write(sudut);
      delay(15); // Ubah nilai ini untuk mengatur kecepatan (semakin besar = semakin lambat)
    }
    
    // 2. Jeda waktu palang tetap terbuka agar kendaraan bisa lewat
    delay(3000); 
    
    // --- 3. MENUTUP PALANG DENGAN LAMBAT (90 ke 0 derajat) ---
    for (int sudut = 90; sudut >= 0; sudut--) {
      palangServo.write(sudut);
      delay(15); // Ubah nilai ini untuk mengatur kecepatan (semakin besar = semakin lambat)
    }
    
  } else if (irMembaca == HIGH) {
    statusKendaraan = false;
  }
  
  jalankanTeksBerjalan();
}

void updateTampilanOLED() {
  display.fillRect(0, 0, 128, 30, SSD1306_BLACK); 
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("SISTEM PARKIR KOTA");
  
  display.setTextSize(2);
  display.setCursor(0, 12);
  display.print("Total: ");
  display.print(jumlahKendaraan);
  display.display();
}

void jalankanTeksBerjalan() {
  display.fillRect(0, 45, 128, 16, SSD1306_BLACK); 
  display.setTextSize(1);
  display.setCursor(posisiX, 48);
  display.print(pesanTeks);
  display.display();
  
  posisiX -= 2; 
  
  if (posisiX < -160) { 
    posisiX = SCREEN_WIDTH;
  }
  
  delay(30); 
}
