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
} state_E;

/**
 * Struct containing all information
 */
typedef struct{
  state_E state;
} flightData_S;

/*****************************************
 *             STATIC VARIABLES          *
 *****************************************/

static flightData_S flightData;

/*****************************************
 *             FUNCTION HEADERS          *
 *****************************************/


/*****************************************
 *           PRIVATE FUNCTIONS           *
 *****************************************/


/*****************************************
 *           PUBLIC FUNCTIONS            *
 *****************************************/
 
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
}
