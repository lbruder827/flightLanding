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

/*****************************************
 *                 DEFINES               *
 *****************************************/

#define COLLECTION_RATE_MS    (uint8_t)100

#define esp_8266_serial       Serial1
#define ESP_8266_SERIAL_BAUD  115200

#define pc_serial             Serial
#define PC_SERIAL_BAUD        9600

#define PIN_ESP8266_CHIP_POWERDOWN  (uint8_t)2U
#define PIN_ESP8266_RESET           (uint8_t)3U

/*****************************************
 *                 TYPEDEFS              *
 *****************************************/
 
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
} flightLanding_data_S;


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

/*****************************************
 *             STATIC VARIABLES          *
 *****************************************/

// State machine and other related information
static flightLanding_data_S flightLanding_data;

// Log file
static const String LOG_FILE = "log.txt";

// Commands initially sent to wifi to setup TCP server
static const String ESP8266_SETUP_CONNECTION = "AT+CIPMUX=1"; // allow multiple connections
static const String ESP8266_SETUP_PORT = "AT+CIPSERVER=1,1336"; // setup TCP server on port 1336
static const String ESP8266_ANYONE_CONNECTED = "AT+CWLIF";  // String to see if anyone is currently connected

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
  // make sure the acceleration and gyroscopic data is calibrated before allowing transition
  // need a way to make this evident to the user
  // https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration

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
        String str = esp_8266_serial.readString();
        pc_serial.println(str);

        if(str.indexOf("COOL") != -1)
        {
            pc_serial.println("I SEE COOL");
        }
        else
        {
            pc_serial.println("NOT RECOGNIZED COMMAND");
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

/**
 * @brief run the AT commands required to setup the TCP server on the device.
 * 
 * @return TRUE if it was setup successfully, FALSE if there was an error
 */
static bool flightLanding_private_setupTCPServer(void)
{
  const uint32_t RESPONSE_DELAY_MS = 1000U;
  uint32_t current_time_ms;

  // clear response
  (void)esp_8266_serial.readString();
  
  esp_8266_serial.println(ESP8266_SETUP_CONNECTION); // enable multiple connections
  current_time_ms = millis();
  
  // delay RESPONSE_DELAY_MS milliseconds
  while((millis() - current_time_ms) < RESPONSE_DELAY_MS);

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
  
  // delay RESPONSE_DELAY_MS milliseconds
  while((millis() - current_time_ms) < RESPONSE_DELAY_MS);

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
  
  // initialize flightLanding_data struct and state
  flightLanding_data.present_state = STATE_WIFI_ON;
  flightLanding_data.desired_state = STATE_WIFI_ON;

  // turn on wifi
  pinMode(PIN_ESP8266_CHIP_POWERDOWN, OUTPUT);// CH_PD
  pinMode(PIN_ESP8266_RESET, OUTPUT);// RST
  digitalWrite(PIN_ESP8266_RESET, HIGH);

  // Give esp8266 time to turn on
  delay(1000);

  flightLanding_private_turnOnWifi(true);

  // Setup serial connection
  esp_8266_serial.begin(ESP_8266_SERIAL_BAUD);
  pc_serial.begin(PC_SERIAL_BAUD);

  // Run setup commands for TCP server on startup
  tcp_server_running = flightLanding_private_setupTCPServer();

  pc_serial.println(tcp_server_running);
  
  if(tcp_server_running == false)
  {
    // log to error file
    // try again here?
    // maybe loop until its running or we reach a count
  }
    
    // open log file
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
