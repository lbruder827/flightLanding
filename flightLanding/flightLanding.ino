
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
#include <SFE_BMP180.h>
#include <SPI.h>
#include <SD.h>

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

#define PIN_BNO055_CHIP_SELECT      10

#define PIN_LED_RED                     23
#define PIN_LED_GREEN                   22
#define PIN_LED_BLUE                    21

#define IMU_MAX_CALIBRATION_VALUE       3

#define SD_CARD_MAX_FILES             255
#define DATA_FILE_NUM_DECIMALS          4

/*****************************************
 *                 TYPEDEFS              *
 *****************************************/

/**
 * Enumerations for fading LEDs
 */
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
  
  bool wifi_ok;
  bool imu_ok;
  bool pressure_ok;
  bool sd_ok;

  double baseline_pressure;

  File data_file;
  bool data_file_open;
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

static bool flightLanding_private_setupTCPServer(void);
static void flightLanding_private_turnOnWifi(bool on);
static void flightLanding_private_fadeLED(led_pins_E ledPin);
static double getPressure(void);

/*****************************************
 *             STATIC VARIABLES          *
 *****************************************/

// State machine and other related information
static flightLanding_data_S flightLanding_data;
static led_information_S led_information;

// IMU and pressure sensor
Adafruit_BNO055 bno = Adafruit_BNO055();
SFE_BMP180 pressure;

// SD Card
static Sd2Card card;

// Log file
static const String LOG_FILE = "log.txt";
static File log_file;

/*
 * ESP8266 commands
 */
static const String ESP8266_SETUP_CONNECTION = "AT+CIPMUX=1"; // allow multiple connections
static const String ESP8266_SETUP_PORT = "AT+CIPSERVER=1,1336"; // setup TCP server on port 1336
static const String ESP8266_ANYONE_CONNECTED = "AT+CWLIF";  // String to see if anyone is currently connected
static const String ESP8266_SEND_STRING = "AT+CIPSEND=0,"; // String to send through wifi

static const String LOG_FILE_EXTENSION = ".csv";
static const String CSV_COLUMN_NAMES = "Time(ms),Altitude(m),Euler_X(deg),Euler_Y(deg),Euler_Z(deg),LinearAccel_X(m/s^2),LinearAccel_Y(m/s^2),LinearAccel_Z(m/s^2),Gyroscopic_X(rad/s),Gyroscopic_Y(rad/s),Gyroscopic_Z(rad/s),Accelerometer_X(m/s^2),Accelerometer_Y(m/s^2),Accelerometer_Z(m/s^2),Gravity_vector_X(m/s^2),Gravity_vector_Y(m/s^2),Gravity_vector_Z(m/s^2)\n";

/*****************************************
 *           PRIVATE FUNCTIONS           *
 *                                       *
 *****************************************/


/*****************************************
 *           STATE TRANSITIONS           *
 *****************************************/

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

//  if(flightLanding_data.imu_ok == true && 
//     flightLanding_data.all_sensors_calibrated == true && buttonpressed)
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

/*****************************************
 *            STATE MACHINE              *
 *****************************************/

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
        // Turn on wifi
        flightLanding_private_turnOnWifi(true);

        if(flightLanding_data.data_file_open == true)
        {
          flightLanding_data.data_file.flush();
          flightLanding_data.data_file.close();
          flightLanding_data.data_file_open = false;
        }
      }
      
      /*
       * Grab calibration values. 
       * If not calibrated, continue to blink green.
       * If calibrated, set the light to solid green
       */
      if(flightLanding_data.all_sensors_calibrated == false)
      {
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        if((gyro == IMU_MAX_CALIBRATION_VALUE)  &&
           (accel == IMU_MAX_CALIBRATION_VALUE) && 
           (mag == IMU_MAX_CALIBRATION_VALUE))
        {
          flightLanding_data.all_sensors_calibrated = true;
          analogWrite(PIN_LED_GREEN, 0);
          analogWrite(PIN_LED_RED, 255);
          analogWrite(PIN_LED_BLUE, 255);
        }
        else
        {
          flightLanding_private_fadeLED(LED_GREEN);
        }
      }

      // process esp8266 serial

      break;

    case STATE_RECORDING:
      if(flightLanding_data.state_transition == true)
      {        
        /*
         * Turn on wifi and grab the baseline pressure
         */
        flightLanding_private_turnOnWifi(false);
        delay(100);
        flightLanding_data.baseline_pressure = getPressure();

        String file_name;
        /*
         * Find a name for the file that doesn't yet exist and open it.
         */
         for(int32_t index = 0; index < SD_CARD_MAX_FILES; index++)
         {
            String file = String(index + LOG_FILE_EXTENSION);
            if(SD.exists(file))
            {
              file_name = file;
            }
            else
            {
              // continue searching
            }
         }
         
         flightLanding_data.data_file = SD.open(file_name);
         flightLanding_data.data_file_open = true;
      }
      
      // grab data every COLLECTION_RATE_MS
      if(millis() - flightLanding_data.last_time_collected_ms > COLLECTION_RATE_MS)
      {
        File data_file = flightLanding_data.data_file;
        
        /**
         * Write the data to the SD card
         */
        uint32_t currentTime_ms = millis();
        data_file.print(currentTime_ms); data_file.print(",");

        // Altitude in meters
        double altitude_m = pressure.altitude(getPressure(), flightLanding_data.baseline_pressure);
        data_file.print(altitude_m,DATA_FILE_NUM_DECIMALS); data_file.print(",");

        // Euler X,Y,Z in degrees
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        data_file.print(euler.x(),DATA_FILE_NUM_DECIMALS); data_file.println(",");
        data_file.print(euler.y(),DATA_FILE_NUM_DECIMALS); data_file.println(",");
        data_file.print(euler.z(),DATA_FILE_NUM_DECIMALS); data_file.println(",");

        // Linear Acceleration X,Y,Z in m/s^2
        imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        data_file.print(linear_accel.x(),DATA_FILE_NUM_DECIMALS); data_file.println(",");
        data_file.print(linear_accel.y(),DATA_FILE_NUM_DECIMALS); data_file.println(",");
        data_file.print(linear_accel.z(),DATA_FILE_NUM_DECIMALS); data_file.println(",");

        // Gyroscopic X,Y,Z in rad/s
        imu::Vector<3> gyroscopic = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        data_file.print(gyroscopic.x(),DATA_FILE_NUM_DECIMALS); data_file.println(",");
        data_file.print(gyroscopic.y(),DATA_FILE_NUM_DECIMALS); data_file.println(",");
        data_file.print(gyroscopic.z(),DATA_FILE_NUM_DECIMALS); data_file.println(",");
        
        // Accelerometer X,Y,Z in m/s^2
        imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        data_file.print(accelerometer.x(),DATA_FILE_NUM_DECIMALS); data_file.println(",");
        data_file.print(accelerometer.y(),DATA_FILE_NUM_DECIMALS); data_file.println(",");
        data_file.print(accelerometer.z(),DATA_FILE_NUM_DECIMALS); data_file.println(",");

        // Gravity vector X,Y,Z in m/s^2
        imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
        data_file.print(gravity.x(),DATA_FILE_NUM_DECIMALS); data_file.println(",");
        data_file.print(gravity.y(),DATA_FILE_NUM_DECIMALS); data_file.println(",");
        data_file.print(gravity.z(),DATA_FILE_NUM_DECIMALS); data_file.println(",");

        data_file.print("\n");
        flightLanding_data.last_time_collected_ms = millis();
      }
      break;

    default:
      // should never get here, log error
      break;
  }

  flightLanding_data.present_state = currentState;
}


/*****************************************
 *            HELPER FUNCTIONS           *
 *****************************************/

/**
 * @brief fade an led
 */
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
  const uint32_t RESPONSE_DELAY_MS = 200U;
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

/**
 * @brief get pressure from BMP180
 *        Function copied from BMP180_altitude_example.ino
 */
static double getPressure(void)
{
  char status;
  double T,P;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);


    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else
        {
          return -1.0;
        }
      }
      else 
      {
          return -1.0;
      }
    }
    else
    {
      return -1.0;
    }
  }
  else 
  {
    return -1.0;
  }
}


/*****************************************
 *           PUBLIC FUNCTIONS            *
 *      ONLY CONTAINS SETUP & LOOP       *
 *****************************************/
 
void setup() 
{  
  /*
   * SETUP SERIAL PORTS
   */
  esp_8266_serial.begin(ESP_8266_SERIAL_BAUD);
  pc_serial.begin(PC_SERIAL_BAUD);

  delay(2000);
  
  pc_serial.println("---Initializing Flight Landing System---\n");
  
  /*
   * INITILIZE DATA STRUCT
   */
  flightLanding_data.present_state = STATE_WIFI_ON;
  flightLanding_data.desired_state = STATE_WIFI_ON;
  flightLanding_data.all_sensors_calibrated = false;
  flightLanding_data.last_time_collected_ms = 0U;

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

  pc_serial.println("Turning on wifi, IMU, SD, and pressure sensor");
  pc_serial.println("Wifi server setup on port 1336, IP address 192.168.4.1\n");

  /*
   * TURN ON WIFI
   */
  flightLanding_private_turnOnWifi(true);
  delay(500);

  /*
   * Setup wifi, IMU, pressure sensor, and SD card
   */
  flightLanding_data.wifi_ok = flightLanding_private_setupTCPServer();  
  flightLanding_data.imu_ok = bno.begin();
  flightLanding_data.pressure_ok = pressure.begin();
  flightLanding_data.sd_ok = SD.begin(PIN_BNO055_CHIP_SELECT);

  // Give everything time to turn on
  delay(500);

  bno.setExtCrystalUse(true);

  pc_serial.println("--- Initialization Status ---");
  pc_serial.print("WIFI: ");            pc_serial.println(flightLanding_data.wifi_ok ?     "OK" : "ERROR");
  pc_serial.print("IMU: ");             pc_serial.println(flightLanding_data.imu_ok ?      "OK" : "ERROR");
  pc_serial.print("Pressure Sensor: "); pc_serial.println(flightLanding_data.pressure_ok ? "OK" : "ERROR");
  pc_serial.print("SD Card Reader: ");  pc_serial.println(flightLanding_data.sd_ok ?       "OK" : "ERROR");

  pc_serial.println("");

  pc_serial.println("--- Opening log file --- ");
  
  log_file = SD.open("log.txt", FILE_WRITE);

  if(log_file)
  {
    pc_serial.println("Succesfully opened log file");
  }
  else
  {
    pc_serial.println("Error opening log file");
  }

  pc_serial.println("Data to be collected\n");

  pc_serial.println(CSV_COLUMN_NAMES);

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

/**
 * Sending data through wifi
 */

  // Echo what is sent into console
  //      if(pc_serial.available())
  //      {
  //        esp_8266_serial.println(pc_serial.readString());
  //      }
  
//        if(esp_8266_serial.available())
//        {
//          String send_str = esp_8266_serial.readString();
//  
//          // echo client
//  #if ECHO
//          // Header to send
//          String send_header = String(ESP8266_SEND_STRING) + String(send_str.length());
//          // Send the header and wait for the '>'
//          esp_8266_serial.println(send_header);
//          uint32_t currTime = millis();
//          while((millis() - currTime) < 1000U);
//          // need to check for > here
//          // send the string
//          esp_8266_serial.println(send_str);
//        
//          currTime = millis();
//          while((millis() - currTime) < 1000U);
//          // clear the buffer
//          (void)esp_8266_serial.readString();
//  #endif
//        }
