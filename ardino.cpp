#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // Use your correct address

void setup() {
  Serial.begin(9600);     // Start serial communication
  lcd.init();             // Initialize LCD
  lcd.backlight();        // Turn on backlight
  lcd.setCursor(0, 0);
  lcd.print("Waiting...");
}

void loop() {
  static String message = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      displayMessage(message);
      message = "";
    } else {
      message += c;
    }
  }
}

void displayMessage(String msg) {
  lcd.clear();

  int line = 0;
  int start = 0;

  while (start < msg.length() && line < 4) {
    lcd.setCursor(0, line);
    String lineText = msg.substring(start, min(start + 20, msg.length()));

    // Pad the line with spaces if it's less than 20 chars
    while (lineText.length() < 20) {
      lineText += ' ';
    }

    lcd.print(lineText);
    start += 20;
    line++;
  }
}
