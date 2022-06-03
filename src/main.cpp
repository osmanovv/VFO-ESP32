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

volatile uint32_t currentFrequency = 7055000;
uint32_t minFrequency = 100000;
uint32_t maxFrequency = 160000000;

// Si5351 correction value
// measure a 10 MHz signal from one of the clock outputs
// (in Hz, or better resolution if you can measure it),
// scale it to parts-per-billion, then use it in the set_correction()
// method in future use of this particular reference oscillator
uint32_t frequencyCorrection = 0;

// predefined steps range: 500Hz, 1kHz, 10kHz
uint16_t steps[] = {500, 1000, 10000};
uint8_t stepsLength = (sizeof(steps) / sizeof(uint16_t));
uint8_t currentStep = 0; // starts with 500Hz

// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(SSD1306_ADDRESS, SDA, SCL);

/**
 * encoder routines
 */

// we only need direction, not the encoder's counter
volatile int8_t _encoderDirection = 0;
volatile bool _rotaryEncoderChanged = false;

/**
 * @brief Get the current step value from predefined steps in '500Hz, 1kHz, 10kHz'
 * 
 * @return uint16_t selected encoder step value from `steps` range {500Hz, 1kHz, 10kHz}
 */
uint16_t getEncoderStep()
{
  return steps[currentStep];
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
  _encoderDirection = 0;
}

// /**
//  * @brief Encoder ISR
//  * 
//  */
void IRAM_ATTR encoderISR(){
  static unsigned long lastTimeChanged = 0;
  // eliminate debouncing
  if (millis() - lastTimeChanged < 150)
  {
    return;
  }
  lastTimeChanged = millis();

  if(digitalRead(ROTARY_ENCODER_A_PIN) == digitalRead(ROTARY_ENCODER_B_PIN)) {
    //Clockwise
    _encoderDirection = 1;
  } else {
    //Counter Clockwise
    _encoderDirection = -1;
  }

  _rotaryEncoderChanged = true;
}

/**
 * @brief Encoder's button ISR
 * 
 */
void IRAM_ATTR encoderButtonISR(){
  rotary_onButtonClick();
}
/**
 * encoder routines end
 */


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
 * @brief Reads rotary encoder direction and set up Si5351 changed frequency
 * 
 */
void rotary_loop()
{
  if (_rotaryEncoderChanged)
  {
    _rotaryEncoderChanged = false;

    currentFrequency += (_encoderDirection * getEncoderStep());

    Serial.print("Value: ");
    Serial.println(currentFrequency);
    si5351_SetupCLK0(currentFrequency, SI5351_DRIVE_STRENGTH_4MA);
  }
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
  
  // encoder initialization
	pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
	pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(ROTARY_ENCODER_A_PIN, encoderISR, CHANGE);
  
  // encoder's button initialization
  pinMode(ROTARY_ENCODER_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(ROTARY_ENCODER_BUTTON_PIN, encoderButtonISR, FALLING);

  // setup default frequency
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