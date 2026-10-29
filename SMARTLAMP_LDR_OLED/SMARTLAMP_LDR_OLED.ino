#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // Lebar display OLED dalam piksel
#define SCREEN_HEIGHT 64 // Tinggi display OLED dalam piksel

// Deklarasi OLED yang terhubung via I2C (Pin Reset -1 karena biasanya tidak dipakai)
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Definisikan Pin
const int pinLDR = A0;
const int pinLED1 = 2;
const int pinLED2 = 3;

// Batas nilai sensor untuk menentukan Siang/Malam (Sesuaikan dengan LDR Anda)
const int batasCahaya = 400; 

// --- BITMAP LOGO MATAHARI (16x16 piksel) ---
const unsigned char logo_matahari[] PROGMEM = {
  0x00, 0x00, 0x02, 0x40, 0x02, 0x40, 0x0c, 0x30, 0x13, 0xc8, 0x3e, 0x7c, 0x7c, 0x3e, 0x6d, 0xb6,
  0x6d, 0xb6, 0x7c, 0x3e, 0x3e, 0x7c, 0x13, 0xc8, 0x0c, 0x30, 0x02, 0x40, 0x02, 0x40, 0x00, 0x00
};

// --- BITMAP LOGO BULAN (16x16 piksel) ---
const unsigned char logo_bulan[] PROGMEM = {
  0x00, 0x00, 0x03, 0xc0, 0x0f, 0x00, 0x1e, 0x00, 0x3c, 0x00, 0x38, 0x00, 0x70, 0x00, 0x70, 0x06,
  0x70, 0x0e, 0x70, 0x1c, 0x38, 0x38, 0x3c, 0x70, 0x1f, 0xe0, 0x0f, 0xc0, 0x03, 0x80, 0x00, 0x00
};

// Variabel untuk mengontrol teks bergerak manual agar tidak memblokir loop utama
int textX = SCREEN_WIDTH; 
String pesanTerakhir = "";

void setup() {
  // Inisialisasi komunikasi Serial dengan Baud Rate 9600 bps
  Serial.begin(9600);
  Serial.println("--- Sistem IoT Smart Lamp City Dimulai ---");

  pinMode(pinLED1, OUTPUT);
  pinMode(pinLED2, OUTPUT);
  
  // Inisialisasi OLED dengan alamat I2C 0x3C
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println("Gagal menginisialisasi layar OLED!");
    for(;;); // Jika gagal, kunci program di sini
  }
  
  display.clearDisplay();
  display.setTextWrap(false); // Matikan pembungkusan teks agar teks bisa berjalan ke kiri
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  // Baca nilai analog dari sensor LDR (0 - 1023)
  int nilaiLDR = analogRead(pinLDR);
  
  String pesanPagi = "Selamat beraktivitas masyarakat kota, tetap semangat menjalani hari!   ";
  String pesanMalam = "Selamat tidur masyarakat kota...   ";
  String pesanAktif = "";

  // LOGIKA KONTROL SMART LAMP & MONITOR
  if (nilaiLDR > batasCahaya) {
    // KONDISI: MALAM (Cahaya rendah)
    digitalWrite(pinLED1, HIGH); // Lampu Kota Menyala
    digitalWrite(pinLED2, HIGH);
    pesanAktif = pesanMalam;
    
    // Tampilkan logo Bulan di sisi atas (koordinat x=56, y=5)
    display.clearDisplay();
    display.drawBitmap(56, 5, logo_bulan, 16, 16, SSD1306_WHITE);
    
    // Mengirim data ke Serial Monitor
    Serial.print("Sensor LDR: ");
    Serial.print(nilaiLDR);
    Serial.println(" -> KONDISI: MALAM (Lampu Menyala)");
  } 
  else {
    // KONDISI: SIANG (Cahaya terang)
    digitalWrite(pinLED1, LOW);  // Lampu Kota Mati
    digitalWrite(pinLED2, LOW);
    pesanAktif = pesanPagi;
    
    // Tampilkan logo Matahari di sisi atas (koordinat x=56, y=5)
    display.clearDisplay();
    display.drawBitmap(56, 5, logo_matahari, 16, 16, SSD1306_WHITE);
    
    // Mengirim data ke Serial Monitor
    Serial.print("Sensor LDR: ");
    Serial.print(nilaiLDR);
    Serial.println(" -> KONDISI: SIANG (Lampu Mati)");
  }

  // LOGIKA TEKS BERGERAK (SCROLLING TEXT)
  if (pesanAktif != pesanTerakhir) {
    textX = SCREEN_WIDTH;
    pesanTerakhir = pesanAktif;
  }

  // Tampilkan text pada baris bawah (y=40)
  display.setTextSize(1);
  display.setCursor(textX, 40);
  display.print(pesanAktif);
  display.display();

  // Geser teks ke arah kiri
  textX -= 3; // Ubah nilai pengurangan untuk mengatur kecepatan gerak teks

  // Jika teks sudah keluar sepenuhnya dari layar sebelah kiri, reset posisinya ke kanan luar
  int panjangTeksPiksel = pesanAktif.length() * 6;
  if (textX < -panjangTeksPiksel) {
    textX = SCREEN_WIDTH;
  }

  delay(30); // Delay kecil untuk stabilitas pergerakan teks dan pembacaan serial
}
