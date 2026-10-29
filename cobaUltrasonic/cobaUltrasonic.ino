// // Definisi Pin Sensor Kiri
// const int trigPinLeft = 5;
// const int echoPinLeft = 18;

// // Definisi Pin Sensor Depan
// const int trigPinFront = 19;
// const int echoPinFront = 21;

// // Definisi Pin Sensor Kanan
// const int trigPinRight = 22;
// const int echoPinRight = 23;

// // Variabel untuk menyimpan durasi dan jarak
// long durationLeft, durationFront, durationRight;
// int distanceLeft, distanceFront, distanceRight;

// // Fungsi untuk membaca jarak dari sensor HC-SR04
// int readDistance(int trigPin, int echoPin) {
//   // Pastikan Trig low terlebih dahulu
//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);
  
//   // Kirim pulsa High selama 10 mikrodetik
//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);
  
//   // Baca durasi pantulan pulsa Echo (timeout set ke 30000 mikrodetik)
//   long duration = pulseIn(echoPin, HIGH, 30000);
  
//   // Hitung jarak dalam centimeter (kecepatan suara = 0.034 cm/us)
//   int distance = duration * 0.034 / 2;
  
//   // Jika di luar jangkauan atau timeout, beri nilai default (misal 999 cm)
//   if (distance == 0) {
//     distance = 999; 
//   }
  
//   return distance;
// }

// void setup() {
//   // Inisialisasi Serial Monitor
//   Serial.begin(115200);
  
//   // Pengaturan mode pin sensor Kiri
//   pinMode(trigPinLeft, OUTPUT);
//   pinMode(echoPinLeft, INPUT);
  
//   // Pengaturan mode pin sensor Depan
//   pinMode(trigPinFront, OUTPUT);
//   pinMode(echoPinFront, INPUT);
  
//   // Pengaturan mode pin sensor Kanan
//   pinMode(trigPinRight, OUTPUT);
//   pinMode(echoPinRight, INPUT);
  
//   Serial.println("--- Tes 3 Sensor HC-SR04 Robot Autonomous ---");
// }

// void loop() {
//   // Membaca jarak dari ketiga sensor
//   distanceLeft  = readDistance(trigPinLeft, echoPinLeft);
//   delay(10); // Jeda singkat agar antar sensor tidak saling menginterferensi gelombang
  
//   distanceFront = readDistance(trigPinFront, echoPinFront);
//   delay(10);
  
//   distanceRight = readDistance(trigPinRight, echoPinRight);
  
//   // Menampilkan data ke Serial Monitor
//   Serial.print("Kiri: ");
//   Serial.print(distanceLeft);
//   Serial.print(" cm | ");
  
//   Serial.print("Depan: ");
//   Serial.print(distanceFront);
//   Serial.print(" cm | ");
  
//   Serial.print("Kanan: ");
//   Serial.print(distanceRight);
//   Serial.println(" cm");
  
//   // Contoh Logika Dasar Pergerakan Robot Autonomous (Bisa kamu modifikasi nanti)
//   /*
//   if (distanceFront < 20) {
//     if (distanceLeft > distanceRight) {
//       Serial.println("-> Aksi: Belok Kiri (Ada rintangan di depan & kanan lebih sempit)");
//     } else {
//       Serial.println("-> Aksi: Belok Kanan (Ada rintangan di depan & kiri lebih sempit)");
//     }
//   } else {
//     Serial.println("-> Aksi: Maju Lurus (Jalan depan aman)");
//   }
//   */

//   delay(300); // Update data setiap 300ms
// }

#define trigPin 8
#define echoPin 9

void setup() {
  Serial.begin(9600);           // Memulai komunikasi Serial Monitor
  pinMode(trigPin, OUTPUT);     // Set pin Trig sebagai OUTPUT
  pinMode(echoPin, INPUT);      // Set pin Echo sebagai INPUT
}

void loop() {
  long durasi;
  float jarak;

  // Bersihkan pin Trig
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Kirim sinyal HIGH selama 10 mikrodetik
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Baca waktu pantulan pada pin Echo
  durasi = pulseIn(echoPin, HIGH);

  // Hitung jarak (kecepatan suara ~340 m/s dibagi 2 untuk perjalanan bolak-balik)
  jarak = (durasi / 2.0) * 0.0343; // Hasil dalam satuan cm

  // Tampilkan hasil ke Serial Monitor
  Serial.print("Jarak = ");
  Serial.print(jarak);
  Serial.println(" cm");

  delay(1000); // Jeda pembacaan 1 detik
}
