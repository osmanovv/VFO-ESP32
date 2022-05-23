/**
 * Another VFO System for ESP32 Ver 0.01
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
#include <si5351.h>

// OLED I2C address
#define SSD1306_ADDRESS 0x3C

// encoder pins
#define ROTARY_ENCODER_A_PIN 32
#define ROTARY_ENCODER_B_PIN 33
#define ROTARY_ENCODER_BUTTON_PIN 25
#define ROTARY_ENCODER_VCC_PIN -1 // microcontroler VCC
// encoder steps: depending on your encoder - try 1,2 or 4 to get expected behaviour
#define ROTARY_ENCODER_STEPS 4

volatile uint32_t currentFrequency = 7055000;
uint32_t minFrequency = 100000;
uint32_t maxFrequency = 160000000;

// Si5351 correction value
// measure a 10 MHz signal from one of the clock outputs
// (in Hz, or better resolution if you can measure it),
// scale it to parts-per-billion, then use it in the set_correction()
// method in future use of this particular reference oscillator
uint32_t frequencyCorrection = 0;

// predefined steps range: 5Hz, 50Hz, 100Hz, 1kHz, 10kHz
uint16_t steps[] = {5, 50, 100, 1000, 10000};
uint8_t stepsLength = (sizeof(steps) / sizeof(uint16_t));
uint8_t currentStep = 1; // starts with 50Hz, not 5Hz

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
 * @brief Draws encoder step
 * 
 * @param step in Hz
 */
void drawStep(uint32_t step) {
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  char buffer[10];
  if (step >= 1000)
  {
    display.drawStringf(0, 24, buffer, "%d kHz", step/1000);
  }
  else
  {
    display.drawStringf(0, 24, buffer, "%d Hz", step);
  }
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
  if (millis() - lastTimePressed < 300)
  {
    return;
  }
  lastTimePressed = millis();

  // change to the next step value
  currentStep = (currentStep + 1)  % stepsLength;
  // reset encoder value as we start count with different step
  rotaryEncoder.setEncoderValue(0);
}

/**
 * @brief Get the current step value from predefined steps in '5Hz, 50Hz, 100Hz, 1kHz, 10kHz'
 * 
 * @return uint16_t selected encoder step value from `steps` range {5Hz, 50Hz, 100Hz, 1kHz, 10kHz}
 */
uint16_t getEncoderStep()
{
  return steps[currentStep];
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
    currentFrequency += ((rotaryEncoder.readEncoder() >= 0 ? 1: -1) * getEncoderStep());

    // reset encoder value as we only need the direction
    rotaryEncoder.setEncoderValue(0);

    // Serial.print("Value: ");
    // Serial.println(currentFrequency);
    si5351_SetupCLK0(currentFrequency, SI5351_DRIVE_STRENGTH_4MA);
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
  // lowering the CPU frequency to reduce power consumption
  setCpuFrequencyMhz(40);
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
  // encoder changes values from -1 to 1 and we use encoder step value to increment the frequency
  rotaryEncoder.setBoundaries(-10, 10, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder.setEncoderValue(0);

  si5351_Init(frequencyCorrection, SDA, SCL);
  
  // set ititial frequency @ ~7 dBm
  si5351_SetupCLK0(currentFrequency, SI5351_DRIVE_STRENGTH_4MA);

  // Enable CLK0
  si5351_EnableOutputs(1<<0);
}

/**
 * @brief Loop routine
 * 
 */
void loop() {
  display.clear();

  display.setContrast(65);

  drawFreq(currentFrequency);
  drawStep(getEncoderStep());

  drawTime();

  // write the buffer to the display
  display.display();

  // handle rotary encoder value
  rotary_loop();
  
  delay(100);
}