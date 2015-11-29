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

/*****************************************
 *                 TYPEDEFS              *
 *****************************************/
 
/**
 * State enumerations
 */
typedef enum{
  STATE_OFF, STATE_WIFI_ON, STATE_RECORDING_ON 
} flightLanding_state_E;

/**
 * Struct containing all information
 */
typedef struct{
  flightLanding_state_E present_state;
  flightLanding_state_E desired_state;
  bool state_transition;
} flightLanding_data_S;

/*****************************************
 *             FUNCTION HEADERS          *
 *****************************************/

// State machine
static void flightLanding_private_processData(void);
static void flightLanding_private_getDesiredState(void);
static void flightLanding_private_setCurrentState(void);

// Tranisitions
static bool flightLanding_private_allowTransitionOffToOn(void);


/*****************************************
 *             STATIC VARIABLES          *
 *****************************************/

static flightLanding_data_S flightLanding_data;

/*****************************************
 *           PRIVATE FUNCTIONS           *
 *****************************************/

static bool flightLanding_private_allowTransitionOffToOn(void)
{
  bool allowTransition = false;

  return allowTransition;
}

/**
 * Process inputs 
 */
static void flightLanding_private_processData(void)
{
  int x = 1;
}

/**
 * Find the desired state based on inputs
 */
static void flightLanding_private_getDesiredState(void)
{
  flightLanding_state_E presentState = flightLanding_data.present_state;

  switch(presentState)
  {
    case STATE_OFF:
    
      // JUST AN EXAMPLE
      if(flightLanding_private_allowTransitionOffToOn() == true)
      {
        presentState = STATE_WIFI_ON;
      }
      else
      {
        // keep state
      }
      
      break;

    default:
      // log error, should never get here
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
    case STATE_OFF:
      if(flightLanding_data.state_transition == true)
      {
        // do transition specific behavior in here
      }

      // do general state behavior here

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
      // initialize flightLanding_data
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
