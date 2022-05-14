/**
 * Another VFO System for ESP32-DevKitC  Ver 0.01
 * by Vladislav Osmanov / R2BFG
 * 2022-05-08
 * 
 * Hardware Configuration for ESP32-DevKitC 38 pins
 * GPIO: Connection
 * 21:  I2C OLED SSD1306 SDA
 * 22:  I2C OLED SSD1306 SCL
 * 32:  Rotary Encoder A
 * 33:  Rotary Encoder B
 * 25:  Rotary Encoder Button
 * 21:  Si5351A SDA
 * 22:  Si5351A SCL
 */

#include <Arduino.h>
#include <Preferences.h>
#include <SSD1306Wire.h>
#include <AiEsp32RotaryEncoder.h>
#include "pins_arduino.h"
#include "Orbitron_Medium_font.h"

// OLED I2C address
#define SSD1306_ADDRESS 0x3C

// encoder pins
#define ROTARY_ENCODER_A_PIN 32
#define ROTARY_ENCODER_B_PIN 33
#define ROTARY_ENCODER_BUTTON_PIN 25
#define ROTARY_ENCODER_VCC_PIN -1 // microcontroler VCC
// encoder steps: depending on your encoder - try 1,2 or 4 to get expected behaviour
#define ROTARY_ENCODER_STEPS 4

uint32_t currentFrequency = 7055000;
uint32_t minFrequency = 100000;
uint32_t maxFrequency = 160000000;

uint32_t rotaryEncoderAcceleration = 250;

// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(SSD1306_ADDRESS, SDA, SCL);

// Initialize the encoder
AiEsp32RotaryEncoder rotaryEncoder(
  ROTARY_ENCODER_A_PIN,
  ROTARY_ENCODER_B_PIN,
  ROTARY_ENCODER_BUTTON_PIN,
  ROTARY_ENCODER_VCC_PIN,
  ROTARY_ENCODER_STEPS);

/**
 * @brief Draws only the frequncy on the screen
 * 
 * @param freq current frequency
 */
void drawFreq(uint32_t freq) {
  display.setFont(Orbitron_Medium_24);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  char buffer[12];
  display.drawStringf(0, 0, buffer, "%d.%03d,%02d",  freq/1000000, (freq/1000)%1000, (freq/10)%100);
}

/**
 * @brief Draws millis from the start
 * 
 */
void drawTime() {
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 54, String(millis()));
}

/**
 * @brief Handles the rotary encoder button press
 * 
 */
void rotary_onButtonClick()
{
  static unsigned long lastTimePressed = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 500)
  {
    return;
  }
  lastTimePressed = millis();

  // TODO: change step value
}

/**
 * @brief Reads rotary encoder value
 * 
 */
void rotary_loop()
{
  //dont print anything unless value changed
  if (rotaryEncoder.encoderChanged())
  {
    currentFrequency = rotaryEncoder.readEncoder();
    // Serial.print("Value: ");
    // Serial.println(currentFrequency);
  }

  if (rotaryEncoder.isEncoderButtonClicked())
  {
    rotary_onButtonClick();
  }
}

/**
 * @brief Encoder ISR
 * 
 */
void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

/**
 * @brief Initial setup after launch
 * 
 */
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();


  // Initialising the UI will init the display too.
  display.init();

  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  
  //we must initialize rotary encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  bool circleValues = false;
  rotaryEncoder.setBoundaries(minFrequency, maxFrequency, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder.setEncoderValue(currentFrequency);
  rotaryEncoder.setAcceleration(rotaryEncoderAcceleration);

}

/**
 * @brief Loop routine
 * 
 */
void loop() {
  display.clear();

  display.setContrast(65);
  //display.setBrightness();
  //display.setContrast(162, 31, 0);

  drawFreq(currentFrequency);

  drawTime();

  // write the buffer to the display
  display.display();

  // handle rotary encoder value
  rotary_loop();
  
  delay(100);
}