// const int trigPin = 9;
// const int echoPin = 10;

// long duration;
// int  distance;
// void setup() {
// pinMode(trigPin, OUTPUT);
// pinMode(echoPin, INPUT);
// Serial.begin(115200);
// }

// void  loop() 
// {
//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(trigPin,  HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);
//   duration  = pulseIn(echoPin, HIGH);
//   distance= duration*0.034/2;
//   Serial.print("Distance:  ");
//   Serial.println(distance);
// }

// Definisi pin ESP32 ke sensor HC-SR04
const int trigPin = 5;  
const int echoPin = 18; 

// Kecepatan suara di udara (0.034 cm per mikrodetik)
const float SOUND_SPEED = 0.034; 

long duration;
float distanceCm;

void setup() {
  // Inisialisasi komunikasi Serial Monitor
  Serial.begin(115200); 

  // Konfigurasi pin
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("Sistem Cek Sensor Ultrasonik HC-SR04 + ESP32 Dimulai...");
}

void loop() {
  // Bersihkan pin trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Kirim sinyal ultrasonik selama 10 mikrodetik
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Membaca durasi pantulan pulsa dari echoPin (timeout 30.000 µs / 30 ms)
  duration = pulseIn(echoPin, HIGH, 30000);

  // Hitung jarak dalam sentimeter
  distanceCm = duration * SOUND_SPEED / 2;

  // Tampilkan hasil di Serial Monitor
  if (duration == 0) {
    Serial.println("Peringatan: Di luar jangkauan atau tidak ada pantulan!");
  } else {
    Serial.print("Jarak: ");
    Serial.print(distanceCm);
    Serial.println(" cm");
  }

  // Jeda pembacaan setiap 1 detik
  delay(1000);
}
