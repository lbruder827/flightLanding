
/**
 * Author: Lucas Bruder
 * Filename: flightLanding.ino
 * Last Modified: 11/28/2015
 * 
 * Main state machine for the flight landing system. 
 * 
 * 
 */

/*****************************************
 *                 INCLUDES              *
 *****************************************/
 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

/*****************************************
 *                 DEFINES               *
 *****************************************/

#define ECHO 1

#define COLLECTION_RATE_MS         (100)
#define BNO055_SAMPLERATE_DELAY_MS (100)

#define esp_8266_serial       Serial1
#define ESP_8266_SERIAL_BAUD  115200

#define pc_serial             Serial
#define PC_SERIAL_BAUD        9600

#define PIN_ESP8266_CHIP_POWERDOWN  2
#define PIN_ESP8266_RESET           3

#define PIN_LED_RED                     23
#define PIN_LED_GREEN                   22
#define PIN_LED_BLUE                    21

#define IMU_MAX_CALIBRATION_VALUE       3
/*****************************************
 *                 TYPEDEFS              *
 *****************************************/

typedef enum{
  LED_RED,
  LED_GREEN,
  LED_BLUE,  
 } led_pins_E;
/**
 * State enumerations
 */
typedef enum{
  /*
   * First enters this state when turned on
   */
  STATE_WIFI_ON,
  /*
   * Recording data, wifi is off
   */
  STATE_RECORDING,
  /*
   * DEBUG state
   */
   STATE_DEBUG
} flightLanding_state_E;

/**
 * Struct containing all information
 */
typedef struct{
  // State related information
  flightLanding_state_E present_state;
  flightLanding_state_E desired_state;
  bool state_transition;

  // Last time we measured data
  uint32_t last_time_collected_ms;

  bool all_sensors_calibrated;
} flightLanding_data_S;

/**
 * Struct containing fade amounts and brightnesses for all LEDs
 */
typedef struct{
  int red_led_brightness;
  int red_led_fade_amount;
  int green_led_brightness;
  int green_led_fade_amount;
  int blue_led_brightness;
  int blue_led_fade_amount;
} led_information_S;


/*****************************************
 *             FUNCTION HEADERS          *
 *****************************************/

// State machine
static void flightLanding_private_processData(void);
static void flightLanding_private_getDesiredState(void);
static void flightLanding_private_setCurrentState(void);

// Tranisitions
static bool flightLanding_private_allowTransitionWifiOnToRecording(void);
static bool flightLanding_private_allowTransitionRecordingToWifiOn(void);
static bool flightLanding_private_allowTransitionWifiOnToDebug(void);
static bool flightLanding_private_allowTransitionDebugToWifiOn(void);

static bool flightLanding_private_setupTCPServer(void);
static void flightLanding_private_turnOnWifi(bool on);
static void flightLanding_private_fadeLED(led_pins_E ledPin);

/*****************************************
 *             STATIC VARIABLES          *
 *****************************************/

// State machine and other related information
static flightLanding_data_S flightLanding_data;
static led_information_S led_information;
Adafruit_BNO055 bno = Adafruit_BNO055();

// Log file
static const String LOG_FILE = "log.txt";

// Commands initially sent to wifi to setup TCP server
static const String ESP8266_SETUP_CONNECTION = "AT+CIPMUX=1"; // allow multiple connections
static const String ESP8266_SETUP_PORT = "AT+CIPSERVER=1,1336"; // setup TCP server on port 1336
static const String ESP8266_ANYONE_CONNECTED = "AT+CWLIF";  // String to see if anyone is currently connected
static const String ESP8266_SEND_STRING = "AT+CIPSEND=0,"; // String to send through wifi, %d is the number of bytes
/*****************************************
 *           PRIVATE FUNCTIONS           *
 *****************************************/

/**
 *  @brief  Check to see if we can transition from debugging the device back to normal operation
 *  
 *  @return TRUE if transition from debug state to wifi state is allowed
 *          FALSE if we should stay in the debug state
 */
static bool flightLanding_private_allowTransitionDebugToWifiOn(void)
{
  bool allowTransition = false;

  return allowTransition;
}

/**
 *  @brief  Check to see if we can transition from wifi on/normal operation to debug mode
 *  
 *  @return TRUE if transition from normal operation to debugging the device
 *          FALSE if we should stay in the wifi state
 */
static bool flightLanding_private_allowTransitionWifiOnToDebug(void)
{
  bool allowTransition = false;

  // This would be entered if we logged into telnet server and entered in special debug command 

  return allowTransition;
}

/**
 *  @brief  Check to see if we can transition from wifi on to recording data
 *  
 *  @return TRUE if transition from wifi on to recording
 *          FALSE if we should stay in the wifi state
 */
static bool flightLanding_private_allowTransitionWifiOnToRecording(void)
{
  bool allowTransition = false;

  // make a new file on the SD card if transition is allowed
  // check for button press and flightLanding_data.all_sensors_calibrated

//  if(flightLanding_data.all_sensors_calibrated == true && buttonpressed)
//  {
//    allowTransition = true;
//  }

  return allowTransition;
}

/**
 *  @brief  Check to see if we can transition from recording back to normal wifi on state
 *  
 *  @return TRUE if transition from recording to wifi on state
 *          FALSE if we should stay in the recording state
 */
static bool flightLanding_private_allowTransitionRecordingToWifiOn(void)
{
  bool allowTransition = false;

  // if transition is allowed to turn on wifi, close the file on the SD card

  return allowTransition;
}

/**
 * Process inputs 
 */
static void flightLanding_private_processData(void)
{
  // grab button presses
}

/**
 * Find the desired state based on inputs. Performs state transitions here
 */
static void flightLanding_private_getDesiredState(void)
{
  flightLanding_state_E presentState = flightLanding_data.present_state;

  switch(presentState)
  {
    case STATE_WIFI_ON:
      if(flightLanding_private_allowTransitionWifiOnToRecording() == true)
      {
        presentState = STATE_RECORDING;
      }
      else if(flightLanding_private_allowTransitionWifiOnToDebug() == true)
      {
        presentState = STATE_DEBUG;
      }
      else
      {
        // keep state
      }
      
      break;
      
    case STATE_RECORDING:
      if(flightLanding_private_allowTransitionRecordingToWifiOn() == true)
      {
        presentState = STATE_WIFI_ON;
      }
      else
      {
        // keep state
      }
      break;

    case STATE_DEBUG:
      if(flightLanding_private_allowTransitionDebugToWifiOn() == true)
      {
        presentState = STATE_WIFI_ON;
      }
      else
      {
        // keep state
      }

    default:
      // log error, should never get to this point
    break;
      
  }

  flightLanding_data.desired_state = presentState;
}

/**
 * Set the current state based on the desired state and perform any outputs
 */
static void flightLanding_private_setCurrentState(void)
{
  flightLanding_state_E currentState = flightLanding_data.desired_state;

  switch(currentState)
  {
    case STATE_WIFI_ON:
      if(flightLanding_data.state_transition == true)
      {
        // transition specific behavior

        // Turn on wifi
        flightLanding_private_turnOnWifi(true);
      }

// Echo what is sent into console
//      if(pc_serial.available())
//      {
//        esp_8266_serial.println(pc_serial.readString());
//      }

      if(esp_8266_serial.available())
      {
        String send_str = esp_8266_serial.readString();

        // echo client
#if ECHO
        // Header containing string to send
        String send_header = String(ESP8266_SEND_STRING) + String(send_str.length());
        // Send the header and wait for the '>'
        esp_8266_serial.println(send_header);
        uint32_t currTime = millis();
        while((millis() - currTime) < 1000U);
        // send the string
        esp_8266_serial.println(send_str);
      
        currTime = millis();
        while((millis() - currTime) < 1000U);
        // clear the buffer
        (void)esp_8266_serial.readString();
#endif
      }

      /*
       * Grab calibration values. If not calibrated, continue to blink green.
       */
      if(flightLanding_data.all_sensors_calibrated == false)
      {
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        Serial.print("CALIBRATION: Sys=");
        Serial.print(system, DEC);
        Serial.print(" Gyro=");
        Serial.print(gyro, DEC);
        Serial.print(" Accel=");
        Serial.print(accel, DEC);
        Serial.print(" Mag=");
        Serial.println(mag, DEC);
        if((gyro == IMU_MAX_CALIBRATION_VALUE)  &&
           (accel == IMU_MAX_CALIBRATION_VALUE) && 
           (mag == IMU_MAX_CALIBRATION_VALUE))
        {
          flightLanding_data.all_sensors_calibrated = true;
          digitalWrite(PIN_LED_GREEN, LOW);
          digitalWrite(PIN_LED_RED, HIGH);
          digitalWrite(PIN_LED_BLUE, HIGH);
        }
        else
        {
          flightLanding_private_fadeLED(LED_GREEN);
        }
      }
      

      break;

    case STATE_RECORDING:
      if(flightLanding_data.state_transition == true)
      {
        // transition specific behavior
        
        // turn off wifi
        flightLanding_private_turnOnWifi(false);
      }
      
      // grab data every COLLECTION_RATE_MS
      if(millis() - flightLanding_data.last_time_collected_ms > COLLECTION_RATE_MS)
      {
        // grab data from IMU and barometer
        // write data to SD file
        flightLanding_data.last_time_collected_ms = millis();
      }
      break;

    case STATE_DEBUG:
      if(flightLanding_data.state_transition == true)
      {
        // transition specific behavior
      }
 
      // normal behavior

      break;

    default:
      // should never get here, log error
      break;
  }

  flightLanding_data.present_state = currentState;
}

static void flightLanding_private_fadeLED(led_pins_E ledPin)
{
    uint32_t led_time_ms;

    if(ledPin == LED_GREEN)
    {
      int brightness = led_information.green_led_brightness;
      int fadeamount = led_information.green_led_fade_amount;
      
      analogWrite(PIN_LED_GREEN, brightness);
      brightness += fadeamount;
      
      if(brightness == 0 || brightness == 255)
      {
        fadeamount = -fadeamount;
      }
  
      led_information.green_led_brightness = brightness;
      led_information.green_led_fade_amount = fadeamount;
      led_time_ms = millis();
      while((millis() - led_time_ms) < 5);
    }
    else if(ledPin == LED_BLUE)
    {
      int brightness = led_information.blue_led_brightness;
      int fadeamount = led_information.blue_led_fade_amount;
      
      analogWrite(PIN_LED_BLUE, brightness);
      brightness += fadeamount;
      
      if(brightness == 0 || brightness == 255)
      {
        fadeamount = -fadeamount;
      }
  
      led_information.blue_led_brightness = brightness;
      led_information.blue_led_fade_amount = fadeamount;
      led_time_ms = millis();
      while((millis() - led_time_ms) < 5);
    }
    else if(ledPin == LED_RED)
    {
      int brightness = led_information.red_led_brightness;
      int fadeamount = led_information.red_led_fade_amount;
      
      analogWrite(PIN_LED_RED, brightness);
      brightness += fadeamount;
      
      if(brightness == 0 || brightness == 255)
      {
        fadeamount = -fadeamount;
      }
  
      led_information.red_led_brightness = brightness;
      led_information.red_led_fade_amount = fadeamount;
      led_time_ms = millis();
      while((millis() - led_time_ms) < 5);
    }
}

/**
 * @brief run the AT commands required to setup the TCP server on the device.
 * 
 * @return TRUE if it was setup successfully, FALSE if there was an error
 */
static bool flightLanding_private_setupTCPServer(void)
{
  const uint32_t RESPONSE_DELAY_MS = 500U;
  uint32_t current_time_ms;

  // clear response
  (void)esp_8266_serial.readString();
  
  esp_8266_serial.println(ESP8266_SETUP_CONNECTION); // enable multiple connections
  current_time_ms = millis();

  /* 
   * Delay and do some fancy LED dimming
   */
  while((millis() - current_time_ms) < RESPONSE_DELAY_MS)
  {
    flightLanding_private_fadeLED(LED_GREEN);
  }

  // ensure we get back "OK"
  if(esp_8266_serial.available())
  {
    String esp8266_response = esp_8266_serial.readString();

    pc_serial.println(esp8266_response);
    
    // make sure it returns OK
    if(esp8266_response.indexOf("OK") == -1)
    {
      return false;
    }
    else
    {
      // continue
    }
  }
  else
  {
    // no response, return false
    return false; 
  }

  esp_8266_serial.println(ESP8266_SETUP_PORT); // Setup TCP server and set port
  current_time_ms = millis();
  
  /* 
   * Delay and do some fancy LED dimming
   */
  while((millis() - current_time_ms) < RESPONSE_DELAY_MS)
  {
    flightLanding_private_fadeLED(LED_GREEN);
  }

  // Ensure we get back OK response
  if(esp_8266_serial.available())
  {
    String esp8266_response = esp_8266_serial.readString();

    pc_serial.println(esp8266_response);


    // make sure it returns OK
    if(esp8266_response.indexOf("OK") == -1)
    {
      return false;
    }
    else
    {
      // continue
    }
  }
  else
  {
    // no response, return false
    return false; 
  }
  
  return true;
}

/**
 * @brief Turn on wifi
 * @param on 
 */
static void flightLanding_private_turnOnWifi(bool on)
{
  if(on == true)
  {
    digitalWrite(PIN_ESP8266_CHIP_POWERDOWN, HIGH);
  }
  else
  {
    digitalWrite(PIN_ESP8266_CHIP_POWERDOWN, LOW);
  }
}



/*****************************************
 *           PUBLIC FUNCTIONS            *
 *      ONLY CONTAINS SETUP & LOOP       *
 *****************************************/
 
void setup() 
{
  bool tcp_server_running = false;
  
  /*
   * INITILIZE DATA STRUCT
   */
  flightLanding_data.present_state = STATE_WIFI_ON;
  flightLanding_data.desired_state = STATE_WIFI_ON;
  flightLanding_data.all_sensors_calibrated = false;

  /*
   * INITIALIZE LED STRUCT
   */
   led_information.red_led_brightness = 0;
   led_information.red_led_fade_amount = 0;
   led_information.green_led_brightness = 0;
   led_information.green_led_fade_amount = 5;
   led_information.blue_led_brightness = 0;
   led_information.blue_led_fade_amount = 0;
  /*
   * SETUP SERIAL PORTS
   */
  esp_8266_serial.begin(ESP_8266_SERIAL_BAUD);
  pc_serial.begin(PC_SERIAL_BAUD);

  /*
   * PIN CONFIGURATION
   */
  pinMode(PIN_ESP8266_CHIP_POWERDOWN, OUTPUT); // CH_PD
  pinMode(PIN_ESP8266_RESET,          OUTPUT); // RST
  pinMode(PIN_LED_RED,                OUTPUT);
  pinMode(PIN_LED_GREEN,              OUTPUT);
  pinMode(PIN_LED_BLUE,               OUTPUT);

  /*
   * INITIALIZE PIN STATES
   */
  digitalWrite(PIN_LED_RED,       HIGH);
  digitalWrite(PIN_LED_GREEN,     LOW);
  digitalWrite(PIN_LED_BLUE,      HIGH);
  digitalWrite(PIN_ESP8266_RESET, HIGH);

  /*
   * TURN ON WIFI
   */
  flightLanding_private_turnOnWifi(true);
  delay(500);

  // Run setup commands for TCP server on startup
  tcp_server_running = flightLanding_private_setupTCPServer();
  
  if(tcp_server_running == false)
  {
    // log to error file
    // try again here?
    // maybe loop until its running or we reach a count
    digitalWrite(PIN_LED_RED,       LOW);
    digitalWrite(PIN_LED_GREEN,     HIGH);
    digitalWrite(PIN_LED_BLUE,      HIGH);
    pc_serial.println("error");
  }

  /*
   * SETUP IMU
   */
   if(!bno.begin())
   {
    // log to error here
    digitalWrite(PIN_LED_RED,       LOW);
    digitalWrite(PIN_LED_GREEN,     HIGH);
    digitalWrite(PIN_LED_BLUE,      HIGH);
    pc_serial.println("error");
   }

   delay(1000);
   bno.setExtCrystalUse(true);
}

void loop() 
{
  // process inputs
  flightLanding_private_processData();

  // find the desired state based on inputs
  flightLanding_private_getDesiredState();

  // state transition?
  flightLanding_data.state_transition = (flightLanding_data.desired_state != flightLanding_data.present_state) ? true : false;

  // pc_serial.print("Current state: "); pc_serial.println(flightLanding_data.present_state);
  // set current state
  flightLanding_private_setCurrentState();
}
