#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 

#define OLED_RESET     -1 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Changed to Analog Pin A0 to read raw numbers on the console
const int FLAME_ANALOG_PIN = A0; 

// SENSITIVITY THRESHOLD:
// Flame sensors read LOWER numbers when fire is near (closer to 0) 
// and HIGHER numbers in the dark (closer to 1023).
// If your console prints a number LOWER than this threshold, it triggers the fire alarm.
const int FIRE_THRESHOLD = 650; 

void setup() {
  // Initialize console communication at 9600 baud rate
  Serial.begin(9600);
  
  // Note: Analog pins do not strictly require pinMode, but it's good practice
  pinMode(FLAME_ANALOG_PIN, INPUT);

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("OLED Allocation Failed!"));
    for(;;); 
  }
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  Serial.println("--- Flame Sensor Console Monitor Started ---");
}

void loop() {
  // Read the raw analog input value (0 to 1023)
  int rawFlameValue = analogRead(FLAME_ANALOG_PIN);

  // --- CONSOLE OUTPUT ---
  // This prints the exact value to your Arduino IDE Serial Monitor
  Serial.print("Raw Sensor Value: ");
  Serial.print(rawFlameValue);
  
  display.clearDisplay(); 
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("SYSTEM STATUS:");
  display.println("---------------------");

  // Determine status based on the raw value vs your threshold
  if (rawFlameValue < FIRE_THRESHOLD) {
    Serial.println(" -> [STATUS: FIRE DETECTED!]");
    
    display.setTextSize(2);
    display.setCursor(0, 25);
    display.println("FIRE");
    display.println("DETECTED!");
  } 
  else {
    Serial.println(" -> [STATUS: SAFE]");
    
    display.setTextSize(1);
    display.setCursor(0, 25);
    display.println("NO FIRE DETECTED");
    
    display.setTextSize(2);
    display.setCursor(0, 42);
    display.println("[ SAFE ]");
  }

  display.display(); 
  delay(300); // Check and print data every 300 milliseconds
}
