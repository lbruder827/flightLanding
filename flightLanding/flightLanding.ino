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

#define COLLECTION_RATE_MS  (uint8_t)100


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
  STATE_RECORDING   // Recording data, wifi is off
} flightLanding_state_E;

/**
 * Struct containing all information
 */
typedef struct{
  // State related information
  flightLanding_state_E present_state;
  flightLanding_state_E desired_state;
  bool state_transition;

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


/*****************************************
 *             STATIC VARIABLES          *
 *****************************************/

static flightLanding_data_S flightLanding_data;

/*****************************************
 *           PRIVATE FUNCTIONS           *
 *****************************************/

static bool flightLanding_private_allowTransitionWifiOnToRecording(void)
{
  bool allowTransition = false;

  // make a new file on the SD card if transition is allowed
  // make sure the acceleration and gyroscopic data is calibrated before allowing transition
  // need a way to make this evident to the user
  // https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration

  return allowTransition;
}

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
 * Find the desired state based on inputs
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
  flightLanding_state_E desiredState = flightLanding_data.desired_state;

  switch(desiredState)
  {
    case STATE_WIFI_ON:
      if(flightLanding_data.state_transition == true)
      {
        // transition specific behavior
        
        // turn on wifi

      }

      // do general state behavior here
      // have wifi file take care of this for different commands

      break;

    case STATE_RECORDING:
      if(flightLanding_data.state_transition == true)
      {
        // transition specific behavior
        
        // turn off wifi
        // turn on IMU
      }
      
      // grab data every COLLECTION_RATE_MS
      if(millis() - flightLanding_data.last_time_collected_ms > COLLECTION_RATE_MS)
      {
        // grab data from IMU and barometer
        // write data to SD file
        flightLanding_data.last_time_collected_ms = millis();
      }
      break;

    default:
      // should never get here, log error
      break;
  }

  flightLanding_data.desired_state = desiredState;
}

/*****************************************
 *           PUBLIC FUNCTIONS            *
 *      ONLY CONTAINS SETUP & LOOP       *
 *****************************************/
 
void setup() 
{
      // initialize flightLanding_data struct and state
}

void loop() 
{
  // process inputs
  flightLanding_private_processData();

  // find the desired state based on inputs
  flightLanding_private_getDesiredState();

  // state transition?
  flightLanding_data.state_transition = (flightLanding_data.desired_state != flightLanding_data.present_state) ? true : false;

  // set current state
  flightLanding_private_setCurrentState();
  
}
