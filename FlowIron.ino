/**** THIS IS A WORK IN PROGRESS AND NOT YET A FULLY FUNCTIONING CODE BASE ****/
/*******************************************************************************
  Title: FlowIron Reflow Hotplate
  Version: 0.0.1
  Date: 07.09.2023
  Author: Adriaan Bernstein

  The code is derived from the Tiny Reflow Controller by Rocket Scream Electronics.
  Original Repository: https://github.com/rocketscream/TinyReflowController/
  Original Author: Lim Phang Moh
********************************************************************************

  Disclaimer:
  ===========
  This repository contains files pertaining to a personal project involving mains voltage, which is hazardous. 
  Ensure you're aware of risks and take precautions. Use or reference this content at your own risk. 
  I assume no liability for any consequences arising from its use.
  
  Overview
  ========
  This project converts a clothing iron into a reflow soldering hotplate. 
  
  Reflow Curves
  =============

  Lead-Free Reflow Curve
  ----------------------
  Temperature (Degree Celcius)                 Magic Happens Here!
  245-|                                               x  x
      |                                            x        x
      |                                         x              x
      |                                      x                    x
  200-|                                   x                          x
      |                              x    |                          |   x
      |                         x         |                          |       x
      |                    x              |                          |
  150-|               x                   |                          |
      |             x |                   |                          |
      |         x     |                   |                          |
      |     x         |                   |                          |
  30 -| x             |                   |                          |
      |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
      | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _
                                                                 Time (Seconds)

  Leaded Reflow Curve (Kester EP256)
  ----------------------------------
  Temperature (Degree Celcius)         Magic Happens Here!
  219-|                                       x  x
      |                                    x        x
      |                                 x              x
  180-|                              x                    x
      |                         x    |                    |   x
      |                    x         |                    |       x
  150-|               x              |                    |           x
      |             x |              |                    |
      |         x     |              |                    |
      |     x         |              |                    |
  30 -| x             |              |                    |
      |<  60 - 90 s  >|<  60 - 90 s >|<   60 - 90 s      >|
      | Preheat Stage | Soaking Stage|   Reflow Stage     | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ _
                                                                 Time (Seconds)
  
  License
  =======
  The parts of this code that are based on the original from Rocket Scream Electronics are
  covered by the Creative Commons Share Alike v4.0 license
  (http://creativecommons.org/licenses/by-sa/4.0/). 
  Please refer to the original repository for full license details. 

  Modifications from Original
  ===========================
  - Adapted for Arduino IoT Nano 33
  - Use Thermistor instead of Thermocouple
  - Uses SH1106 128x64 OLED display in place of SSD1306 
  - Transitioned to an encoder-based input & control scheme
  - Add manual temperature profile

  Required Libraries
  ==================
  - Arduino PID Library (bc fork; https://github.com/drf5n/Arduino-PID-Library)
  - Adafruit NeoPixel (https://github.com/adafruit/Adafruit_NeoPixel)
  - Adafruit GFX Library (https://github.com/adafruit/Adafruit-GFX-Library)
  - Adafruit SH110X Library (https://github.com/adafruit/Adafruit_SH110x)
  - EncoderStepCounter Library (https://github.com/M-Reimer/EncoderStepCounter)

*******************************************************************************/

// ***** INCLUDES *****
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <PID_v1_bc.h>
#include <EncoderStepCounter.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE {
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS {
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

typedef enum SWITCH {
  SWITCH_NONE,
  SWITCH_1
} switch_t;

typedef enum DEBOUNCE_STATE {
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

typedef enum REFLOW_PROFILE {
  REFLOW_PROFILE_LEADFREE,
  REFLOW_PROFILE_LEADED,
  REFLOW_PROFILE_MANUAL
} reflowProfile_t;

// ***** CONSTANTS *****

// ***** GENERAL PROFILE CONSTANTS *****
#define PROFILE_TYPE_ADDRESS 0
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5

// ***** LEAD FREE PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_LF 200
#define TEMPERATURE_REFLOW_MAX_LF 250
#define SOAK_MICRO_PERIOD_LF 9000

// ***** LEADED PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_PB 180
#define TEMPERATURE_REFLOW_MAX_PB 224
#define SOAK_MICRO_PERIOD_PB 10000

// ***** MANUAL TEMP SELECT CONSTANTS *****
#define TEMPERATURE_SOAK_MAN_OFFSET 45
#define TEMPERATURE_REFLOW_MAN_START 200
#define SOAK_MICRO_PERIOD_MAN 9000

// ***** SWITCH SPECIFIC CONSTANTS *****
#define DEBOUNCE_PERIOD_MIN 100

// ***** DISPLAY SPECIFIC CONSTANTS *****
#define UPDATE_RATE 100
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define X_AXIS_START 18  // X-axis starting position

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "Ready",
  "Pre",
  "Soak",
  "Reflow",
  "Cool",
  "Done!",
  "Hot!",
  "Error"
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8] = {
  140, 146, 146, 140, 128, 128, 128, 128
};

// ***** PIN ASSIGNMENT *****
unsigned char ssrPin = 2;
unsigned char thermistorPin = A7;
unsigned char ledPin = 4;
unsigned char buzzerPin = 5;
unsigned char switchStartStopPin = 3;
unsigned char encA = 10;
unsigned char encB = 11;

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;

int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long updateLcd;
unsigned long timerSoak;
unsigned long buzzerPeriod;
unsigned char soakTemperatureMax;
unsigned char reflowTemperatureMax;
unsigned long soakMicroPeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Reflow profile type
reflowProfile_t reflowProfile;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
switch_t switchValue;
switch_t switchMask;
// Seconds timer
unsigned int timerSeconds;
// Thermocouple fault status //TODO
unsigned char fault;
unsigned int timerUpdate;
unsigned char temperature[SCREEN_WIDTH - X_AXIS_START];
unsigned char x;
// Encoder
signed char lastPos = 0;
signed char posChange;
unsigned int manualTemp;


// Initialize PID Control Object
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Initialize OLED Display Control Object
Adafruit_SH1106G oled = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Initialize Thermistor Control Object
//TODO THERMISTOR INIT

// Create Encoder and LED control Objects
EncoderStepCounter encoder(encA, encB);
Adafruit_NeoPixel statusLed = Adafruit_NeoPixel(1, ledPin, NEO_RGB + NEO_KHZ800);


void setup() {
  // Initialize encoder & attach Interrupts
  encoder.begin();
  attachInterrupt(digitalPinToInterrupt(encA), interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB), interrupt, CHANGE);

  //Hard-code default profile instead of storing last used
  reflowProfile = REFLOW_PROFILE_LEADFREE;

  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  // Buzzer pin initialization
  pinMode(buzzerPin, OUTPUT);
  noTone(buzzerPin);

  // Encoder Backlight LED initialization and turn on upon start-up
  statusLed.begin();
  statusLed.setPixelColor(0, statusLed.Color(255, 0, 0));
  statusLed.show();

  // Initialize thermistor interface
  // TODO THERMISTOR

  // Start-up splash
  tone(buzzerPin, 2000);

  oled.begin(0x3C, true);
  oled.display();
  noTone(buzzerPin);
  delay(1000);
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SH110X_WHITE);
  oled.setCursor(0, 0);
  oled.println(F("      FlowIron"));
  oled.println(F("     Controller"));
  oled.println();
  oled.println(F("      v0.0.1"));
  oled.println();
  oled.println(F("      07-09-23"));
  oled.display();
  delay(2000);
  oled.clearDisplay();

  // Serial communication at 115200 bps
  Serial.begin(115200);

  // Turn off LED
  statusLed.setPixelColor(0, statusLed.Color(0, 0, 0));
  statusLed.show();
  // Set window size
  windowSize = 2000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermistor reading variable
  nextRead = millis();
  // Initialize LCD update timer
  updateLcd = millis();
}

void loop() {
  // Current time
  unsigned long now;

  // Detect Encoder Direction
  signed char pos = encoder.getPosition();
  posChange = 0;
  if (pos < lastPos) {
    posChange = -1;
    lastPos = pos;
  } else if (pos > lastPos) {
    posChange = 1;
    lastPos = pos;
  }

  // Time to read thermistor?
  if (millis() > nextRead) {
    // Read thermistor next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature
    // TODO THERMISTOR
    // input = thermocouple.readThermocoupleTemperature();
    // Check for thermocouple fault
    // fault = thermocouple.readFault();
    input = 24;

    //TODO THERMAL PROTECTION
    /* if ((fault & MAX31856_FAULT_CJRANGE) ||
        (fault & MAX31856_FAULT_TCRANGE) ||
        (fault & MAX31856_FAULT_CJHIGH) ||
        (fault & MAX31856_FAULT_CJLOW) ||
        (fault & MAX31856_FAULT_TCHIGH) ||
        (fault & MAX31856_FAULT_TCLOW) ||
        (fault & MAX31856_FAULT_OVUV) ||
        (fault & MAX31856_FAULT_OPEN))
    { 
      // Illegal operation
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
      Serial.println(F("Error"));
    } */
  }

  if (millis() > nextCheck) {
    // Check input in the next seconds
    nextCheck += SENSOR_SAMPLING_TIME;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON) {
      // Toggle the NeoPixel color as a system heartbeat (alternate between red and off)
      if (statusLed.getPixelColor(0) == statusLed.Color(255, 0, 0)) {
        statusLed.setPixelColor(0, statusLed.Color(0, 0, 0));  // Turn off
      } else {
        statusLed.setPixelColor(0, statusLed.Color(255, 0, 0));  // Turn on with red color
      }
      statusLed.show();
      // Increase seconds timer for reflow curve plot
      timerSeconds++;
      // Send temperature and time stamp to serial
      Serial.print(timerSeconds);
      Serial.print(F(","));
      Serial.print(setpoint);
      Serial.print(F(","));
      Serial.print(input);
      Serial.print(F(","));
      Serial.println(output);
    } else {
      // Turn off LED
      statusLed.setPixelColor(0, statusLed.Color(0,0,0));
      statusLed.show();
    }
  }

  if (millis() > updateLcd) {
    // Update LCD in the next 100 ms
    updateLcd += UPDATE_RATE;

    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setCursor(0, 0);
    oled.print(lcdMessagesReflowStatus[reflowState]);
    oled.setTextSize(1);
    oled.setCursor(74, 0);  //Orig: 115

    if (reflowProfile == REFLOW_PROFILE_LEADFREE) {
      oled.print(F("LF (245C)"));
    } else if (reflowProfile == REFLOW_PROFILE_LEADED) {
      oled.print(F("PB (219C)"));
    } else  //REFLOW_PROFILE_MANUAL
    {
      oled.print(F("MAN (XX)"));
    }

    // Temperature markers
    oled.setCursor(0, 18);
    oled.print(F("250"));
    oled.setCursor(0, 36);
    oled.print(F("150"));
    oled.setCursor(0, 54);
    oled.print(F("50"));
    // Draw temperature and time axis
    oled.drawLine(18, 18, 18, 63, SH110X_WHITE);
    oled.drawLine(18, 63, 127, 63, SH110X_WHITE);
    oled.setCursor(115, 0);

    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR) {
      oled.setCursor(80, 9);
      oled.print(F("TC Error"));
    } else {
      // Right align temperature reading
      if (input < 10) oled.setCursor(91, 9);
      else if (input < 100) oled.setCursor(85, 9);
      else oled.setCursor(80, 9);
      // Display current temperature
      oled.print(input);
      oled.print((char)247);
      oled.print(F("C"));
    }

    if (reflowStatus == REFLOW_STATUS_ON) {
      // We are updating the display faster than sensor reading
      if (timerSeconds > timerUpdate) {
        // Store temperature reading every 3 s
        if ((timerSeconds % 3) == 0) {
          timerUpdate = timerSeconds;
          unsigned char averageReading = map(input, 0, 250, 63, 19);
          if (x < (SCREEN_WIDTH - X_AXIS_START)) {
            temperature[x++] = averageReading;
          }
        }
      }
    }

    unsigned char timeAxis;
    for (timeAxis = 0; timeAxis < x; timeAxis++) {
      oled.drawPixel(timeAxis + X_AXIS_START, temperature[timeAxis], SH110X_WHITE);
    }

    // Update screen
    oled.display();
  }

  // Reflow oven controller state machine
  switch (reflowState) {
    case REFLOW_STATE_IDLE:
      // If oven temperature is still above room temperature
      if (input >= TEMPERATURE_ROOM) {
        reflowState = REFLOW_STATE_TOO_HOT;
      } else {
        // If switch is pressed to start reflow process
        if (switchStatus == SWITCH_1) {
          // Send header for CSV file
          Serial.println(F("Time,Setpoint,Input,Output"));
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;

          // Initialize reflow plot update timer
          timerUpdate = 0;

          for (x = 0; x < (SCREEN_WIDTH - X_AXIS_START); x++) {
            temperature[x] = 0;
          }
          // Initialize index for average temperature array used for reflow plot
          x = 0;

          // Initialize PID control window starting time
          windowStartTime = millis();
          // Ramp up to minimum soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN;
          // Load profile specific constant
          if (reflowProfile == REFLOW_PROFILE_LEADFREE) {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_LF;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_LF;
            soakMicroPeriod = SOAK_MICRO_PERIOD_LF;
          }

          else if (reflowProfile == REFLOW_PROFILE_LEADED) {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_PB;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_PB;
            soakMicroPeriod = SOAK_MICRO_PERIOD_PB;
          } else  //REFLOW_PROFILE_MANUAL
          {
            manualTemp = TEMPERATURE_REFLOW_MAN_START + 0;
            soakTemperatureMax = manualTemp - TEMPERATURE_SOAK_MAN_OFFSET;  //TODO
            reflowTemperatureMax = manualTemp;                              //TODO
            soakMicroPeriod = SOAK_MICRO_PERIOD_MAN;
          }
          // Tell the PID to range between 0 and the full window size
          reflowOvenPID.SetOutputLimits(0, windowSize);
          reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
          // Turn the PID on
          reflowOvenPID.SetMode(AUTOMATIC);
          // Proceed to preheat stage
          reflowState = REFLOW_STATE_PREHEAT;
        }
      }
      break;

    case REFLOW_STATE_PREHEAT:
      reflowStatus = REFLOW_STATUS_ON;
      // If minimum soak temperature is achieve
      if (input >= TEMPERATURE_SOAK_MIN) {
        // Chop soaking period into smaller sub-period
        timerSoak = millis() + soakMicroPeriod;
        // Set less agressive PID parameters for soaking ramp
        reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;
      }
      break;

    case REFLOW_STATE_SOAK:
      // If micro soak temperature is achieved
      if (millis() > timerSoak) {
        timerSoak = millis() + soakMicroPeriod;
        // Increment micro setpoint
        setpoint += SOAK_TEMPERATURE_STEP;
        if (setpoint > soakTemperatureMax) {
          // Set agressive PID parameters for reflow ramp
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
          // Ramp up to first section of soaking temperature
          setpoint = reflowTemperatureMax;
          // Proceed to reflowing state
          reflowState = REFLOW_STATE_REFLOW;
        }
      }
      break;

    case REFLOW_STATE_REFLOW:
      // We need to avoid hovering at peak temperature for too long
      // Crude method that works like a charm and safe for the components
      if (input >= (reflowTemperatureMax - 5)) {
        // Set PID parameters for cooling ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp down to minimum cooling temperature
        setpoint = TEMPERATURE_COOL_MIN;
        // Proceed to cooling state
        reflowState = REFLOW_STATE_COOL;
      }
      break;

    case REFLOW_STATE_COOL:
      // If minimum cool temperature is achieve
      if (input <= TEMPERATURE_COOL_MIN) {
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 1000;
        // Turn on buzzer to indicate completion
        tone(buzzerPin, 2000);
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Proceed to reflow Completion state
        reflowState = REFLOW_STATE_COMPLETE;
      }
      break;

    case REFLOW_STATE_COMPLETE:
      if (millis() > buzzerPeriod) {
        // Turn off buzzer
        noTone(buzzerPin);
        // Reflow process ended
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_TOO_HOT:
      // If oven temperature drops below room temperature
      if (input < TEMPERATURE_ROOM) {
        // Ready to reflow
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_ERROR:
      // Check for thermocouple fault
      //TODO// fault = thermocouple.readFault();
      /*
      // If thermocouple problem is still present
      if ((fault & MAX31856_FAULT_CJRANGE) ||
          (fault & MAX31856_FAULT_TCRANGE) ||
          (fault & MAX31856_FAULT_CJHIGH) ||
          (fault & MAX31856_FAULT_CJLOW) ||
          (fault & MAX31856_FAULT_TCHIGH) ||
          (fault & MAX31856_FAULT_TCLOW) ||
          (fault & MAX31856_FAULT_OVUV) ||
          (fault & MAX31856_FAULT_OPEN))
      {
        // Wait until thermocouple wire is connected
        reflowState = REFLOW_STATE_ERROR;
      }
      else
      { */
      // Clear to perform reflow process
      reflowState = REFLOW_STATE_IDLE;
      //TODO//}
      break;
  }

  // If switch 1 is pressed
  if (switchStatus == SWITCH_1) {
    // If currently reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON) {
      // Button press is for cancelling
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;
    }
  }

  if (reflowState == REFLOW_STATE_IDLE) {
    if (posChange > 0) {  // Encoder rotated clockwise
      // Increment the reflow profile
      switch (reflowProfile) {
        case REFLOW_PROFILE_LEADFREE:
          reflowProfile = REFLOW_PROFILE_LEADED;
          break;
        case REFLOW_PROFILE_LEADED:
          reflowProfile = REFLOW_PROFILE_MANUAL;
          break;
        case REFLOW_PROFILE_MANUAL:
          reflowProfile = REFLOW_PROFILE_LEADFREE;
          break;
      }
    } else if (posChange < 0) {  // Encoder rotated counter-clockwise
      // Decrement the reflow profile
      switch (reflowProfile) {
        case REFLOW_PROFILE_LEADED:
          reflowProfile = REFLOW_PROFILE_LEADFREE;
          break;
        case REFLOW_PROFILE_MANUAL:
          reflowProfile = REFLOW_PROFILE_LEADED;
          break;
        case REFLOW_PROFILE_LEADFREE:
          reflowProfile = REFLOW_PROFILE_MANUAL;
          break;
      }
    }
  }

  // Switch status has been read
  switchStatus = SWITCH_NONE;

  // Simple switch debounce state machine (analog switch)
  switch (debounceState) {
    case DEBOUNCE_STATE_IDLE:
      // No valid switch press
      switchStatus = SWITCH_NONE;

      switchValue = readSwitch();

      // If a switch is pressed
      if (switchValue != SWITCH_NONE) {
        // Keep track of the pressed switch (not needed for one switch but possibly useful in the future)
        switchMask = switchValue;
        // Intialize debounce counter
        lastDebounceTime = millis();
        // Proceed to check validity of button press
        debounceState = DEBOUNCE_STATE_CHECK;
      }
      break;

    case DEBOUNCE_STATE_CHECK:
      switchValue = readSwitch();
      if (switchValue == switchMask) {
        // If minimum debounce period is completed
        if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN) {
          // Valid switch press
          switchStatus = switchMask;
          // Proceed to wait for button release
          debounceState = DEBOUNCE_STATE_RELEASE;
        }
      }
      // False trigger
      else {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;

    case DEBOUNCE_STATE_RELEASE:
      switchValue = readSwitch();
      if (switchValue == SWITCH_NONE) {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;
  }

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON) {
    now = millis();

    reflowOvenPID.Compute();

    if ((now - windowStartTime) > windowSize) {
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if (output > (now - windowStartTime)) digitalWrite(ssrPin, HIGH);
    else digitalWrite(ssrPin, LOW);
  }
  // Reflow oven process is off, ensure oven is off
  else {
    digitalWrite(ssrPin, LOW);
  }
}

switch_t readSwitch(void) {

  int switchAdcValue = 0;

  // Switch connected directly to individual separate pins
  if (digitalRead(switchStartStopPin) == HIGH) return SWITCH_1;

  return SWITCH_NONE;
}

// Call tick on every change interrupt
void interrupt() {
  encoder.tick();
}