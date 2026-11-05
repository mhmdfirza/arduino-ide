// Definisikan pin LED bawaan (pada kebanyakan ESP32 DevKit, berada di GPIO 2)
#define LED_BUILTIN 2

void setup() {
  // Inisialisasi komunikasi Serial dengan baud rate 115200 (opsional untuk monitoring)
  Serial.begin(115200);
  
  // Atur pin LED sebagai OUTPUT
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Nyalakan LED
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("LED ON");
  delay(1000); // Tunggu 1 detik (1000 ms)

  // Matikan LED
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("LED OFF");
  delay(1000); // Tunggu 1 detik (1000 ms)
}
