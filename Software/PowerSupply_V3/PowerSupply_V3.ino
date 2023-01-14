/**
 //
 // PowerSuply V3
 //
 // Control sketch for a two screen, constant voltage, constant current power supply.
 //
 //
 // Author         Isaac Rex
 //
 // Date           12/22
 // Version        0.9
 **/


#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include "ILI9341_t3.h"
#include "font_monofonto.h"
#include "AutoPID.h"

// ********************************** //
// *             Constants          * //
// ********************************** //
#define MEAS_FONT Monofonto_60
#define UNIT_FONT Monofonto_40
#define SET_FONT  Monofonto_32
#define SET_STR   "Set: "
// Serial commands
#define COMMAND_CAL      'c'
#define COMMAND_PID_ON   "pid on"
#define COMMAND_PID_OFF  "pid off"
#define COMMAND_GET_TEMP "temp"

// ********************************** //
// *              DEBUG             * //
// ********************************** //
#define DEBUG_OUTPUT

// ********************************** //
// *              Pins              * //
// ********************************** //
// Pin definintions
#define tft_RST 8
#define fan_pin 22
// Voltage screen pins
#define tft_voltsDC 15
#define tft_voltsCS 21
#define tft_voltsBacklight 23
// Current screen pins
#define tft_ampsDC 10
#define tft_ampsCS 9
#define tft_ampsBacklight 4
// Measurement pins
#define currentSetPin 17
#define voltsPin A11
#define ampsPin A2
#define tempPin A5

// ********************************** //
// *            Constants           * //
// ********************************** //

// Determined emperically. Mapping function is:
// A*exp(B*x). x should be normalzied: x = raw_adc / (2^ADC_NUM_BITS)
const float TEMP_COEFF_A = 119.25;
const float TEMP_COEFF_B = -3.40;

// Target temperature for heatsink in celcius. NJW0281 R_{θJC} = 0.83 °C/W. Each transistor disipates 
// ~ 22V * 1.5A = 33 W. Assuming R_{θCH} ~= 0.5 °C/W and the heastink is at 50 °C then the BJT case is
// at 50 °C + (0.5 °C/W * 33 W) = 67 °C and the junction temp is at 67 °C + (0.83 °C/W * 33 W) = 94 °C.
// The max operating temperature is 150 °C.
double TARGET_TEMP_HEATSINK_C   = 50;
double TEMP_PID_BANGBANG_THRESH = 10; // If temp is more than 10 deg above or below setpoint, set to min or max
uint32_t TEMP_PID_TIME_STEP_MS  = 1000; // Update PID every 1 s

const uint16_t ADC_NUM_BITS        = 16;
const uint16_t ADC_AVERAGE_COUNT   = 1;    // Number of hardware averages
const uint16_t NUM_SAMPLES         = 1000; // Number of readings to average for voltage and current; must be >= 2
const uint16_t UPDATE_DELAY_MS     = 40;   // Set the number of miliseconds to wait between display updates
const float    LEAD_RESISTANCE     = 0.045;
const uint32_t FAN_UPDATE_DELAY_MS = 10; // Time to wait between fan updates
double         FAN_MAX_VAL         = 255;
double         FAN_MIN_VAL         = 0;

/*
*  187500 Hz gives 8-bits real resolution
 * 750000 Hz gives 6-bits real resolution
 * 1500000 Hz gives 5-bits real resulotion
 */
const uint32_t FAN_PWM_FREQ = 187500;
const uint32_t INPUT_BUFF_SIZE = 100;
// Update the display only if the value has changed by at least UPDATE_DELTA
const float    UPDATE_DELTA = 0.001; 

// EEPROM addresses for the calibration data
const uint16_t EEPROM_VOLTS_SLOPE_ADDR  = 0;
const uint16_t EEPROM_VOLTS_OFFSET_ADDR = EEPROM_VOLTS_SLOPE_ADDR + sizeof(float);
const uint16_t EEPROM_AMPS_SLOPE_ADDR   = EEPROM_VOLTS_OFFSET_ADDR + sizeof(float);
const uint16_t EEPROM_AMPS_OFFSET_ADDR   = EEPROM_AMPS_SLOPE_ADDR + sizeof(float);

const uint32_t SPI_SPEED_HZ     = 30000000; 
const uint16_t LCD_WIDTH        = 320;
const uint16_t LCD_HEIGHT       = 240;
const uint16_t BACKGROUND_COLOR = ILI9341_BLACK;
const uint16_t MEAS_FONT_COLOR  = ILI9341_WHITE;
const uint16_t SET_FONT_COLOR   = ILI9341_WHITE;
const uint16_t UNIT_FONT_COLOR  = ILI9341_DARKGREY;
const uint16_t CAL_FONT_COLOR   = ILI9341_OLIVE;

// Determined emperically. It'd be nice to dynamically choose these, but I don't want to spend the time
const uint16_t MEAS_START_POS_X       = 5;
const uint16_t CENTER_TXT_START_POS_Y = 90;
const uint16_t SET_START_POS_X        = 15;
const uint16_t SET_START_POS_Y        = 180;
const uint16_t UNIT_START_POS_X       = 15;
const uint16_t UNIT_START_POS_Y       = 15;
const uint8_t  PRINT_VALUE_BUFF_SIZE  = 7;  // Length is for the string "xx.xxx" plus null
const uint8_t  TFT_RESET_DELAY_MS     = 20; // Hold time for manual TFT reset

// ********************************** //
// *               Types            * //
// ********************************** //
typedef struct {
  ILI9341_t3 display;
  float current_mes_val; // Holds the currently displayed measured value
  char *current_mes_str; 
  float current_set_val; // Holds the currently displayed set value
  char *current_set_str; 
} display_info_t;

typedef struct {
  int   data_arr[NUM_SAMPLES];
  int   idx;
  long  rawsum;
  float val_avg;
  bool  initialized;
} adc_avg_t;

// ********************************** //
// *            Variables           * //
// ********************************** //
adc_avg_t volts_averager;
adc_avg_t amps_averager;
adc_avg_t amps_set_averager;
adc_avg_t temp_averager;

uint16_t set_num_start_pos_x = 0;

float volts_slope;
float volts_offset;
float amps_slope;
float amps_offset;

double  temperature_c = 0;
double  fan_speed = 150;
double  pid_output = 0;
double  kp = 35;
double  ki = 0.6;
double  kd = 0.001;
AutoPID temp_PID(&temperature_c, &TARGET_TEMP_HEATSINK_C, &pid_output, FAN_MIN_VAL, FAN_MAX_VAL, kp, ki, kd);

// ********************************** //
// *            Prototypes          * //
// ********************************** //
void tft_printMeasurement(display_info_t* display, float val);
void tft_printSet(display_info_t* display, float val);
void tft_printUnit(display_info_t* display, const char* unit);
void tft_printCentered(display_info_t* display, const char* str, uint16_t font_color);
void average_update(adc_avg_t* averager, int new_val);
void calibrate();
bool _calibrate_checkForFloat(float *x);
float inline mapVolts(int adc_raw);
float inline mapAmps(int adc_raw);
float inline mapTemp(int adc_raw);

// Utilities
display_info_t tft_volts = { .display = ILI9341_t3(tft_voltsCS, tft_voltsDC), 
                             .current_mes_val = -1.0f,
                             .current_mes_str = (char*) calloc(PRINT_VALUE_BUFF_SIZE, sizeof(char)),
                             .current_set_val = -1.0f,
                             .current_set_str = NULL};

display_info_t tft_amps  = { .display = ILI9341_t3(tft_ampsCS, tft_ampsDC), 
                             .current_mes_val = -1.0f,
                             .current_mes_str = (char*) calloc(PRINT_VALUE_BUFF_SIZE, sizeof(char)),
                             .current_set_val = -1.0f,
                             .current_set_str = (char*) calloc(PRINT_VALUE_BUFF_SIZE, sizeof(char))};


void setup()
{
    // Set up serial port for debugging
    Serial.begin(9600);
    delay(10);  // Time for terminal to connect
    Serial.println("Starting...");

    // ********************************** //
    // *            Pin Modes           * //
    // ********************************** //
    Serial.println("Setting GPIOs");
    // Outputs
    pinMode(tft_voltsBacklight, OUTPUT);
    pinMode(tft_ampsBacklight, OUTPUT);
    pinMode(fan_pin, OUTPUT);
    pinMode(tft_RST, OUTPUT);
    //Inputs
    pinMode(currentSetPin, INPUT);
    pinMode(voltsPin, INPUT);
    pinMode(tempPin, INPUT);
    // Since the dummy who designed the hardware didn't add a RST pin to both
    // LCDs or make sure the CS pins were pulled up, we need to pull them up
    // internally before setting up the LCDs otherwise the second screen won't
    // always init properly. 
    pinMode(tft_voltsCS, INPUT_PULLUP);
    pinMode(tft_ampsCS, INPUT_PULLUP);

    // ********************************** //
    // *            Setup ADC           * //
    // ********************************** //
    char message[40];
    sprintf(message, "Setting ADC Resolution to %d bits...", ADC_NUM_BITS);
    Serial.println(message);
    analogReadAveraging(ADC_AVERAGE_COUNT);
    analogReadRes(ADC_NUM_BITS);

    // ********************************** //
    // *          Load Cal Data         * //
    // ********************************** //
    EEPROM.get(EEPROM_VOLTS_SLOPE_ADDR, volts_slope);
    EEPROM.get(EEPROM_VOLTS_OFFSET_ADDR, volts_offset);
    EEPROM.get(EEPROM_AMPS_SLOPE_ADDR, amps_slope);
    EEPROM.get(EEPROM_AMPS_OFFSET_ADDR, amps_offset);

    // ********************************** //
    // *            Setup LCDs          * //
    // ********************************** //

    // Manually reset both displays.
    Serial.println("Initializing LCDs");
    digitalWrite(tft_RST, HIGH);
    delay(5);
    digitalWrite(tft_RST, LOW);
    delay(TFT_RESET_DELAY_MS);
    digitalWrite(tft_RST, HIGH);
    delay(TFT_RESET_DELAY_MS);
    tft_volts.display.begin();
    tft_amps.display.begin();

    // Setup volts LCD:
    Serial.println("Setting up Volts LCD");
    tft_volts.display.setRotation(3);
    tft_volts.display.setClock(SPI_SPEED_HZ);
    tft_volts.display.fillScreen(BACKGROUND_COLOR);
    digitalWrite(tft_voltsBacklight, HIGH);
    tft_printUnit(&tft_volts, "Volts");

    // Setup amps LCD:
    Serial.println("Setting up Current LCD");
    tft_amps.display.setRotation(1);
    tft_amps.display.setClock(SPI_SPEED_HZ);
    tft_amps.display.fillScreen(BACKGROUND_COLOR);
    tft_printUnit(&tft_amps, "Amps");
    // Draw set current info
    tft_amps.display.setTextColor(SET_FONT_COLOR, BACKGROUND_COLOR);
    tft_amps.display.setCursor(SET_START_POS_X, SET_START_POS_Y);
    tft_amps.display.print(SET_STR);
    digitalWrite(tft_ampsBacklight, HIGH);

    tft_amps.display.setFont(SET_FONT);
    set_num_start_pos_x = SET_START_POS_X + tft_amps.display.measureTextWidth(SET_STR);

    // ********************************** //
    // *             Fan PID            * //
    // ********************************** //
    temp_PID.setBangBang(TEMP_PID_BANGBANG_THRESH);
    temp_PID.setTimeStep(TEMP_PID_TIME_STEP_MS);

    Serial.println("Initialization done.");
    Serial.println("Send 'c' to start calibration.");
}

void loop() {
    static unsigned long last_update_ms = 0;

    // Average temp
    average_update(&temp_averager, analogRead(tempPin));
    // Average volts
    average_update(&volts_averager, analogRead(voltsPin));
    // Average amps
    average_update(&amps_averager, analogRead(ampsPin));
    average_update(&amps_set_averager, analogRead(currentSetPin));

    // Convert the raw sum to the floating point decimal value of the voltage
    float amps = mapAmps(amps_averager.val_avg);
    float volts = mapVolts(volts_averager.val_avg) - amps * LEAD_RESISTANCE;
    float amps_set = amps_set_averager.val_avg / (1 << ADC_NUM_BITS) * 3.0f;
    temperature_c = mapTemp(temp_averager.val_avg);

    // Update PID
    if (!temp_PID.isStopped()) {
        temp_PID.run();
        fan_speed = FAN_MAX_VAL - pid_output;
        analogWrite(fan_pin, fan_speed);
    }

    // Only update the display once every UPDATE_DELAY interval
    if (millis() - last_update_ms > UPDATE_DELAY_MS) {
        // Serial.print("Temperature: ");
        // Serial.println(mapTemp(temp_averager.val_avg));
        // Serial.print("Fanspeed: ");
        // Serial.println(fan_speed);
        // Serial.print("Kp, Ki, Kd: ");
        // Serial.print(kp);
        // Serial.print(", ");
        // Serial.print(ki);
        // Serial.print(", ");
        // Serial.println(kd);

        tft_printMeasurement(&tft_volts, volts);
        tft_printMeasurement(&tft_amps, amps);
        tft_printSet(&tft_amps, amps_set);
        // Grab the current time so we know when to update the screen next
        last_update_ms = millis();
    }

    // Check for a commands
    if (Serial.available() > 0) {
        char *input_buff = (char *) malloc(INPUT_BUFF_SIZE * sizeof(char));
        Serial.readBytesUntil('\n', input_buff, INPUT_BUFF_SIZE);
        // Convert string to lowercase
        char *p = input_buff;
        for ( ; *p; ++p) *p = tolower(*p);

        if (input_buff[0] == COMMAND_CAL) { // Calibrate
            calibrate();
        } else if (strcmp(input_buff, COMMAND_PID_ON) == 0) { // Start PID
            temp_PID.run();
            Serial.println("PID loop started.");
        } else if (strcmp(input_buff, COMMAND_PID_OFF) == 0) { // Stop PID
            temp_PID.stop();
            Serial.println("PID loop stoped.");
            analogWrite(fan_pin, FAN_MIN_VAL);
        } else if (strcmp(input_buff, COMMAND_GET_TEMP) == 0) { // Print measured temp
            Serial.println(temperature_c);
        } else {
            Serial.println("Unrecognized command: ");
            Serial.print(input_buff);
        }
    }
}

/**
Updates the given display with the given measurement. The measurement will be the large
value in the center of the display. This function is much more efficient than printCentered
for updating displayed numbers.

NOTE: Text can get messed up if a negative value is displayed. The black text will overwrite 
some of the new white text. This is fixable, but I chose to just disallow negative 
numbers for now. Any negative value will display as 0.000
**/
void tft_printMeasurement(display_info_t* display, float val) {
  val = max(0, val);
  float last_val = display->current_mes_val;
  float delta = val - last_val;

  if (abs((delta)) >= UPDATE_DELTA) {
    uint16_t cursor_x_current;
    uint16_t cursor_x_new;
    uint16_t char_width;
    uint16_t char_height;

    // Format the value into a string
    char *current_str = display->current_mes_str;
    char *new_str = (char*) malloc(PRINT_VALUE_BUFF_SIZE);
    uint8_t num_decimal_places = (val < 9.9995f) ? 3 : 2;
    sprintf(new_str, "%.*f", num_decimal_places, val);

    display->display.setFont(MEAS_FONT);

    // Calculate the cursor position to center the text
    cursor_x_current = LCD_WIDTH/2 - display->display.measureTextWidth(current_str)/2;
    cursor_x_new = LCD_WIDTH/2 - display->display.measureTextWidth(new_str)/2;

    // Loop through the display string and only redraw updated characters
    for (int i = 0; i < PRINT_VALUE_BUFF_SIZE-1; i++) {
      if (current_str[i] != new_str[i]) { // this char has changed, update
        // Blackout the previous text
        display->display.setCursor(cursor_x_current, CENTER_TXT_START_POS_Y);
        display->display.setTextColor(BACKGROUND_COLOR);
        display->display.drawFontChar(current_str[i]);

        // Draw new text
        display->display.setTextColor(MEAS_FONT_COLOR);
        display->display.setCursor(cursor_x_new, CENTER_TXT_START_POS_Y);
        display->display.drawFontChar(new_str[i]);

        // Update our string tracker
        current_str[i] = new_str[i];
      }
      // Update cursor position
      // For some reason, passing NULL to the height argument causes it to crash...
      display->display.measureChar(new_str[i], &char_width, &char_height);
      cursor_x_current += char_width;
      cursor_x_new += char_width;
      
    }

    display->current_mes_val = val;
  }
}

/**
Updates the given display with the given set value. The set value will be the smaller
value on the bottom of the display.

NOTE: Text can get messed up if a negative value is displayed. The black text will overwrite 
some of the new white text. This is fixable, but I chose to just disallow negative 
numbers for now. Any negative value will display as 0.000.

NOTE: This could be implemented as a one-size-fits-all function with measurement...probably
a better way to do it?
**/
void tft_printSet(display_info_t* display, float val) {
    val = max(0, val);
    float last_val = display->current_set_val;
    float delta = val - last_val;

    if (abs((delta)) >= UPDATE_DELTA) {
      uint16_t cursor_x;
      uint16_t char_width;
      uint16_t char_height;

      // Format the value into a string
      char *current_str = display->current_set_str;
      char *new_str = (char*) malloc(PRINT_VALUE_BUFF_SIZE);
      uint8_t num_decimal_places = (val < 9.9995f) ? 3 : 2;
      sprintf(new_str, "%.*f", num_decimal_places, val);

      display->display.setFont(SET_FONT);
      display->display.setTextColor(SET_FONT_COLOR, BACKGROUND_COLOR); 

      // Calculate the cursor position to center the text
      cursor_x = set_num_start_pos_x;

      // Loop through the display string and only redraw updated characters
      for (int i = 0; i < PRINT_VALUE_BUFF_SIZE-1; i++) {
        if (current_str[i] != new_str[i]) { // this char has changed, update
          // Blackout the previous text
          display->display.setCursor(cursor_x, SET_START_POS_Y);
          display->display.setTextColor(BACKGROUND_COLOR);
          display->display.drawFontChar(current_str[i]);

          // Draw new text
          display->display.setTextColor(SET_FONT_COLOR);
          display->display.setCursor(cursor_x, SET_START_POS_Y);
          display->display.drawFontChar(new_str[i]);

          // Update our string tracker
          current_str[i] = new_str[i];
        }
        // Update cursor position
        // For some reason, passing NULL to the height argument causes it to crash...
        display->display.measureChar(new_str[i], &char_width, &char_height);
        cursor_x += char_width;
      }

      display->current_set_val = val;
    }
}

/** Draws the given string in the upper left corner of the display in UNIT_FONT_COLOR **/
void tft_printUnit(display_info_t* display, const char* unit) {
  display->display.setCursor(UNIT_START_POS_X, UNIT_START_POS_Y);
  display->display.setFont(UNIT_FONT);
  display->display.setTextColor(UNIT_FONT_COLOR);
  display->display.print(unit);
}

/** Draw the given string centered on the display. Use printMeasurement for updating the measured
text values, as it's optimized for doing that. This function is mostly intended to be used with
the calibration routine.
**/
void tft_printCentered(display_info_t* display, const char* str, uint16_t font_color) {
    char *current_str = display->current_mes_str;
    uint16_t cursor_x;

    // Blank current text
    display->display.setFont(MEAS_FONT);
    cursor_x = LCD_WIDTH/2 - display->display.measureTextWidth(current_str)/2;
    display->display.setCursor(cursor_x, CENTER_TXT_START_POS_Y);
    display->display.setTextColor(BACKGROUND_COLOR);
    display->display.print(current_str);

    // Draw new text
    cursor_x = LCD_WIDTH/2 - display->display.measureTextWidth(str)/2;
    display->display.setCursor(cursor_x, CENTER_TXT_START_POS_Y);
    display->display.setTextColor(font_color);
    display->display.print(str);

    strcpy(current_str, str);
}

/** Updates the given average tracker with the given new_val.

NOTE: This could probably be more generalized as a class...but who has time for that? 
**/
void average_update(adc_avg_t* averager, int new_val) {
  if (!averager->initialized) {
    averager->idx = 0;
    averager->rawsum = new_val;
    averager->val_avg = (float) new_val;
    averager->initialized = true;
  }

  averager->rawsum -= averager->data_arr[averager->idx];
	averager->data_arr[averager->idx] = new_val;
	averager->rawsum += new_val;
	averager->idx = (averager->idx + 1) % NUM_SAMPLES;

  averager->val_avg = ((float) averager->rawsum) / NUM_SAMPLES;
}

/** Maps a raw ADC reading to a volts measurement **/
float inline mapVolts(int adc_raw) {
  return (adc_raw * volts_slope + volts_offset);
}


float inline mapAmps(int adc_raw) {
  return (adc_raw * amps_slope + amps_offset);
}

float inline mapTemp(int adc_raw) {
    float adc_normalized = (float) adc_raw / ((float) (1 << ADC_NUM_BITS));
    return (TEMP_COEFF_A * exp(TEMP_COEFF_B * adc_normalized));
}

void calibrate() {
    float raw_volts_low = -1.0f;
    float raw_volts_high = -1.0f;
    float volts_low = -1.0f;
    float volts_high = -1.0f;

    float raw_amps_low = -1.0f;
    float raw_amps_high = -1.0f;
    float amps_low = -1.0f;
    float amps_high = -1.0f;
    
    Serial.println("\nBegin Calibration:");
    tft_printCentered(&tft_volts, "-----", CAL_FONT_COLOR);
    tft_printCentered(&tft_amps, "-----", MEAS_FONT_COLOR);
    /****************************
     *           Volts          *
     ****************************/
    // Get the volts_low value
    Serial.println("Set voltage to minimum setting and enter measured value.");
    bool phase_done = false;
    while(!phase_done) {
        phase_done = _calibrate_checkForFloat(&volts_low);
        average_update(&volts_averager, analogRead(voltsPin));
    }   
    raw_volts_low = volts_averager.val_avg;

    // Get the volts_high value
    Serial.println("Set voltage to maximum setting and enter measured value.");
    phase_done = false;
    while(!phase_done) {
        phase_done = _calibrate_checkForFloat(&volts_high);
        average_update(&volts_averager, analogRead(voltsPin));
    }   
    raw_volts_high = volts_averager.val_avg;

    /****************************
     *           Amps           *
     ****************************/
    tft_printCentered(&tft_volts, "-----", MEAS_FONT_COLOR);
    tft_printCentered(&tft_amps, "-----", CAL_FONT_COLOR);

    // Get the amps_low value
    Serial.println("Set amps to low setting and enter measured value.");
    phase_done = false;
    while(!phase_done) {
        phase_done = _calibrate_checkForFloat(&amps_low);
        average_update(&amps_averager, analogRead(ampsPin));
    }   
    raw_amps_low = amps_averager.val_avg;

    // Get the volts_high value
    Serial.println("Set amps to high setting and enter measured value.");
    phase_done = false;
    while(!phase_done) {
        phase_done = _calibrate_checkForFloat(&amps_high);
        average_update(&amps_averager, analogRead(ampsPin));
    }   
    raw_amps_high = amps_averager.val_avg;

    // Update local variables
    volts_slope = (volts_high - volts_low) / (raw_volts_high - raw_volts_low);
    volts_offset = volts_high - volts_slope * raw_volts_high;

    amps_slope = (amps_high - amps_low) / (raw_amps_high - raw_amps_low);
    amps_offset = amps_high - amps_slope * raw_amps_high;

    Serial.print("Volts slope: ");
    Serial.println(volts_slope * (1 << ADC_NUM_BITS));
    Serial.print("Volts Offset: ");
    Serial.println(volts_offset);

    Serial.print("Amps slope: ");
    Serial.println(amps_slope * (1 << ADC_NUM_BITS));
    Serial.print("Amps Offset: ");
    Serial.println(amps_offset);

    // Store values in EEPROM
    EEPROM.put(EEPROM_VOLTS_SLOPE_ADDR, volts_slope);
    EEPROM.put(EEPROM_VOLTS_OFFSET_ADDR, volts_offset);
    EEPROM.put(EEPROM_AMPS_SLOPE_ADDR, amps_slope);
    EEPROM.put(EEPROM_AMPS_OFFSET_ADDR, amps_offset);

    tft_printCentered(&tft_volts, "", MEAS_FONT_COLOR);
    tft_printCentered(&tft_amps, "", MEAS_FONT_COLOR);
    Serial.print("Calibration complete");
}

/** Checks for Serial data and tries to parse it into a float. Stores the parsed float at
    the location given in x. If the parse succeeds, returns true. 
**/
bool _calibrate_checkForFloat(float *x) {
    if(Serial.available()) {
            float val;
            char *input_buff = (char *) malloc((INPUT_BUFF_SIZE + 1) * sizeof(char));
            // Read the input string
            Serial.readBytesUntil('\n', input_buff, INPUT_BUFF_SIZE);
            // Print the recieved value
            Serial.print("Received: ");
            Serial.println((input_buff));
            // Try converting to a float
            val = atof(input_buff); // Returns 0.00 if invalid...which is dumb
            if (val != 0.0f) {
                val = (val == -1.0f) ? 0.0f : val;
                *x = val;
                return true;
            } else {
                Serial.println("Input not valid. Please enter a valid float. 0.0 is invalid, if you actually meant 0, please enter -1.0");
            }
        }
    return false;
}