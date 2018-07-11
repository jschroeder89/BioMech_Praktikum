#include "userthread.hpp"

#include "global.hpp"

using namespace amiro;
#include <math.h>
extern Global global;

// State machine states
enum states : uint8_t {
	IDLE,
	GO_RIGHT,
	GO_STRAIGHT,
	PARKING,
	PARKING_RIGHT,
	PARKING_LEFT,
	GO_LEFT,
	SPINNING_PARKING,
	SPINNING
};

// Policy
states policy[] = {
  GO_STRAIGHT,
  GO_RIGHT,
  GO_RIGHT,
  GO_STRAIGHT,
  GO_RIGHT,
  GO_STRAIGHT,
  GO_RIGHT,
  GO_STRAIGHT,
  GO_STRAIGHT,
  GO_RIGHT,
  GO_STRAIGHT,
  GO_RIGHT,
  GO_STRAIGHT
};

// The different classes (or members) of color discrimination
// BLACK is the line itselfe
// GREY is the boarder between the line and the surface
// WHITE is the common surface
enum colorMember : uint8_t {
	BLACK=0,
	GREY=1,
	WHITE=2
};

// a buffer for the z-value of the accelerometer
int16_t accel_z;
bool running;

// Get some information about the policy
const int sizeOfPolicy = sizeof(policy) / sizeof(states);
int policyCounter = 0; // Do not change this, it points to the beginning of the policy

// Different speed settings (all values in "rounds per minute")
const int speedFactor = 1;
const int raceFactor = 1*speedFactor;
const int rpmForward[2] = {25*speedFactor+raceFactor,25*speedFactor+raceFactor};
const int rpmSoftLeft[2] = {15*speedFactor,25*speedFactor};
const int rpmHardLeft[2] = {10*speedFactor,25*speedFactor};
const int rpmSoftRight[2] = {rpmSoftLeft[1],rpmSoftLeft[0]};
const int rpmHardRight[2] = {rpmHardLeft[1],rpmHardLeft[0]};
const int rpmTurnLeft[2] = {-10*speedFactor, 10*speedFactor};
const int rpmTurnRight[2] = {rpmTurnLeft[1],rpmTurnLeft[0]};
const int rpmHalt[2] = {0, 0};
int lastTurn[2] = {0,0};
int turn[2] = {0,0};
const int maxSpeed = 60;

// Definition of the fuzzyfication function
//  | Membership
// 1|_B__   G    __W__
//  |    \  /\  /
//  |     \/  \/
//  |_____/\__/\______ Sensor values
// SEE MATLAB SCRIPT "fuzzyRule.m" for adjusting the values
// All values are "raw sensor values"
/* Use these values for white ground surface (e.g. paper) */

const int blackStartFalling = 0x1000; // Where the black curve starts falling
const int blackOff = 0x1800; // Where no more black is detected
const int whiteStartRising = 0x2800; // Where the white curve starts rising
const int whiteOn = 0x6000; // Where the white curve has reached the maximum value
const int greyMax = (whiteOn + blackStartFalling) / 2; // Where grey has its maximum
const int greyStartRising = blackStartFalling; // Where grey starts rising
const int greyOff = whiteOn; // Where grey is completely off again

/* Use these values for gray ground surfaces */
/*
const int blackStartFalling = 0x1000; // Where the black curve starts falling
const int blackOff = 0x2800; // Where no more black is detected
const int whiteStartRising = 0x4000; // Where the white curve starts rising
const int whiteOn = 0x5000; // Where the white curve starts rising
const int greyMax = (whiteOn + blackStartFalling) / 2; // Where grey has its maximum
const int greyStartRising = blackStartFalling; // Where grey starts rising
const int greyOff = whiteOn; // Where grey is completely off again
*/

int vcnl4020AmbientLight[4] = {0};
int vcnl4020Proximity[4] = {0};

// Border for the discrimination between black and white
const int discrBlackWhite = 16000; // border in "raw sensor values"
// Discrimination between black and white (returns BLACK or WHITE)
// The border was calculated by a MAP-decider
colorMember discrimination(int value) {
	if (value < discrBlackWhite)
		return BLACK;
	else
		return WHITE;
}

// Copy the speed from the source to the target array
void copyRpmSpeed(const int (&source)[2], int (&target)[2]) {
	target[constants::DiWheelDrive::LEFT_WHEEL] = source[constants::DiWheelDrive::LEFT_WHEEL];
	target[constants::DiWheelDrive::RIGHT_WHEEL] = source[constants::DiWheelDrive::RIGHT_WHEEL];
}

// Fuzzyfication of the sensor values
void fuzzyfication(int sensorValue, float (&fuzziedValue)[3]) {
	if (sensorValue < blackStartFalling ) {
		// Only black value
		fuzziedValue[BLACK] = 1.0f;
		fuzziedValue[GREY] = 0.0f;
		fuzziedValue[WHITE] = 0.0f;
	} else if (sensorValue > whiteOn ) {
		// Only white value
		fuzziedValue[BLACK] = 0.0f;
		fuzziedValue[GREY] = 0.0f;
		fuzziedValue[WHITE] = 1.0f;
	} else if ( sensorValue < greyMax) {
		// Some greyisch value between black and grey

		// Black is going down
		if ( sensorValue > blackOff) {
			fuzziedValue[BLACK] = 0.0f;
		} else {
			fuzziedValue[BLACK] = static_cast<float>(sensorValue-blackOff) / (blackStartFalling-blackOff);
		}

		// Grey is going up
		if ( sensorValue < greyStartRising) {
			fuzziedValue[GREY] = 0.0f;
		} else {
			fuzziedValue[GREY] = static_cast<float>(sensorValue-greyStartRising) / (greyMax-greyStartRising);
		}

		// White is absent
		fuzziedValue[WHITE] = 0.0f;

	} else if ( sensorValue >= greyMax) {
		// Some greyisch value between grey white

		// Black is absent
		fuzziedValue[BLACK] = 0.0f;

		// Grey is going down
		if ( sensorValue < greyOff) {
			fuzziedValue[GREY] = static_cast<float>(sensorValue-greyOff) / (greyMax-greyOff);
		} else {
			fuzziedValue[GREY] = 0.0f;
		}

		// White is going up
		if ( sensorValue < whiteStartRising) {
			fuzziedValue[WHITE] = 0.0f;
		} else {
			fuzziedValue[WHITE] = static_cast<float>(sensorValue-whiteStartRising) / (whiteOn-whiteStartRising);
		}
	}
}

// Return the color, which has the highest fuzzy value
colorMember getMember(float (&fuzzyValue)[3]) {
	colorMember member;

	if (fuzzyValue[BLACK] > fuzzyValue[GREY])
		if (fuzzyValue[BLACK] > fuzzyValue[WHITE])
			member = BLACK;
		else
			member = WHITE;
	else
		if (fuzzyValue[GREY] > fuzzyValue[WHITE])
			member = GREY;
		else
			member = WHITE;

	return member;
}

// Get a crisp output for the steering commands
void defuzzyfication(colorMember (&member)[4], int (&rpmFuzzyCtrl)[2]) {

	// all sensors are equal
	if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == member[constants::DiWheelDrive::PROX_FRONT_LEFT] &&
	    member[constants::DiWheelDrive::PROX_FRONT_LEFT] == member[constants::DiWheelDrive::PROX_FRONT_RIGHT] &&
	    member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == member[constants::DiWheelDrive::PROX_WHEEL_RIGHT]) {
		// something is wrong -> stop
		copyRpmSpeed(rpmHalt, rpmFuzzyCtrl);
	// both front sensor detect a line
	} else if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == BLACK &&
	    member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == BLACK) {
		// straight
		copyRpmSpeed(rpmForward, rpmFuzzyCtrl);
	// exact one front sensor detects a line
	} else if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == BLACK ||
	           member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == BLACK) {
		// soft correction
		if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == GREY) {
			// soft right
			if (lastTurn[0] >= 2){
				turn[0] =  25*speedFactor*3;
				turn[1] = (25-(10/(lastTurn[0])))*speedFactor*3;
				copyRpmSpeed(turn, rpmFuzzyCtrl);
				global.robot.setLightColor(3, Color(Color::LAVENDER));
			} else {
				copyRpmSpeed(rpmSoftRight, rpmFuzzyCtrl);
				global.robot.setLightColor(3, Color(Color::BLACK));
			}
			lastTurn[1]++;
		} else if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == WHITE) {
			// hard right
			copyRpmSpeed(rpmHardRight, rpmFuzzyCtrl);
			lastTurn[1] = 1;
		} else if (member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == GREY) {
			// soft left
			if (lastTurn[1] >= 2){
				turn[0] = (25-(10/(lastTurn[0])))*speedFactor*3;
				turn[1] = 25*speedFactor*3;
				copyRpmSpeed(turn, rpmFuzzyCtrl);
			} else {
				copyRpmSpeed(rpmSoftLeft, rpmFuzzyCtrl);	
			}
			lastTurn[0]++;
		} else if (member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == WHITE) {
			// hard left
			copyRpmSpeed(rpmHardLeft, rpmFuzzyCtrl);
			lastTurn[0] = 1;
		}
	// both wheel sensors detect a line
	} else if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == BLACK &&
	           member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == BLACK) {
		// something is wrong -> stop
		copyRpmSpeed(rpmHalt, rpmFuzzyCtrl);
	// exactly one wheel sensor detects a line
	} else if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == BLACK ||
	           member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == BLACK) {
		if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == BLACK) {
			// turn left
			copyRpmSpeed(rpmTurnLeft, rpmFuzzyCtrl);
		} else if (member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == BLACK) {
			// turn right
			copyRpmSpeed(rpmTurnRight, rpmFuzzyCtrl);
		}
	// both front sensors may detect a line
	} else if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == GREY &&
	           member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == GREY) {
		if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == GREY) {
			// turn left
			copyRpmSpeed(rpmTurnLeft, rpmFuzzyCtrl);
		} else if (member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == GREY) {
			// turn right
			copyRpmSpeed(rpmTurnRight, rpmFuzzyCtrl);
		}
	// exactly one front sensor may detect a line
	} else if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == GREY ||
	           member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == GREY) {
		if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == GREY) {
			// turn left
			copyRpmSpeed(rpmTurnLeft, rpmFuzzyCtrl);
		} else if (member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == GREY) {
			// turn right
			copyRpmSpeed(rpmTurnRight, rpmFuzzyCtrl);
		}
	// both wheel sensors may detect a line
	} else if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == GREY &&
	           member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == GREY) {
		// something is wrong -> stop
		copyRpmSpeed(rpmHalt, rpmFuzzyCtrl);
	// exactly one wheel sensor may detect a line
	} else if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == GREY ||
	           member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == GREY) {
		if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == GREY) {
			// turn left
			copyRpmSpeed(rpmTurnLeft, rpmFuzzyCtrl);
		} else if (member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == GREY) {
			// turn right
			copyRpmSpeed(rpmTurnRight, rpmFuzzyCtrl);
		}
	// no sensor detects anything 
	} else {
		// line is lost -> stop
		copyRpmSpeed(rpmHalt, rpmFuzzyCtrl);
	}

	return;
}

Color memberToLed(colorMember member) {
	switch (member) {
		case BLACK:
			return Color(Color::GREEN);
		case GREY:
			return Color(Color::YELLOW);
		case WHITE:
			return Color(Color::RED);
		default:
			return Color(Color::WHITE);
	}
}

// Line following by a fuzzy controler
void lineFollowing(int (&proximity)[4], int (&rpmFuzzyCtrl)[2]) {
	// FUZZYFICATION
	// First we need to get the fuzzy value for our 3 values {BLACK, GREY, WHITE}
	float leftWheelFuzzyMemberValues[3], leftFrontFuzzyMemberValues[3], rightFrontFuzzyMemberValues[3], rightWheelFuzzyMemberValues[3];
	fuzzyfication(proximity[constants::DiWheelDrive::PROX_WHEEL_LEFT], leftWheelFuzzyMemberValues);
	fuzzyfication(proximity[constants::DiWheelDrive::PROX_FRONT_LEFT], leftFrontFuzzyMemberValues);
	fuzzyfication(proximity[constants::DiWheelDrive::PROX_FRONT_RIGHT], rightFrontFuzzyMemberValues);
	fuzzyfication(proximity[constants::DiWheelDrive::PROX_WHEEL_RIGHT], rightWheelFuzzyMemberValues);

	// INFERENCE RULE DEFINITION
	// Get the member for each sensor
	colorMember member[4];
	member[constants::DiWheelDrive::PROX_WHEEL_LEFT] = getMember(leftWheelFuzzyMemberValues);
	member[constants::DiWheelDrive::PROX_FRONT_LEFT] = getMember(leftFrontFuzzyMemberValues);
	member[constants::DiWheelDrive::PROX_FRONT_RIGHT] = getMember(rightFrontFuzzyMemberValues);
	member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] = getMember(rightWheelFuzzyMemberValues);
	
	// visualize sensors via LEDs
	global.robot.setLightColor(constants::LightRing::LED_WNW, memberToLed(member[constants::DiWheelDrive::PROX_WHEEL_LEFT]));
	global.robot.setLightColor(constants::LightRing::LED_NNW, memberToLed(member[constants::DiWheelDrive::PROX_FRONT_LEFT]));
	global.robot.setLightColor(constants::LightRing::LED_NNE, memberToLed(member[constants::DiWheelDrive::PROX_FRONT_RIGHT]));
	global.robot.setLightColor(constants::LightRing::LED_ENE, memberToLed(member[constants::DiWheelDrive::PROX_WHEEL_RIGHT]));

//	chprintf((BaseSequentialStream*) &SD1, "Left: BLACK: %f, GREY: %f, WHITE: %f\r\n", leftFuzzyMemberValues[BLACK], leftFuzzyMemberValues[GREY], leftFuzzyMemberValues[WHITE]);

	// DEFUZZYFICATION
	defuzzyfication(member, rpmFuzzyCtrl);
}

// Set the speed by the array
void setRpmSpeed(const int (&rpmSpeed)[2]) {
	global.motorcontrol.setTargetRPM(rpmSpeed[constants::DiWheelDrive::LEFT_WHEEL] * 1000000, rpmSpeed[constants::DiWheelDrive::RIGHT_WHEEL] * 1000000);
}

// Get the next policy rule
states getNextPolicy() {
	// If the policy is over, start again
	if (policyCounter >= sizeOfPolicy)
		policyCounter = 3;

	return policy[policyCounter++];
}


UserThread::UserThread() :
  chibios_rt::BaseStaticThread<USER_THREAD_STACK_SIZE>()
{
}

UserThread::~UserThread()
{
}

msg_t
UserThread::main()
{
/*
	while(!this->shouldTerminate()) {
		chprintf((BaseSequentialStream*)&global.sercanmux1, "Hello World!\n");
		this->sleep(MS2ST(1000));
	}
*/
	
	// * SETUP
	 
	int rpmFuzzyCtrl[2] = {0};
	int initialVals[4]{0};
	int foo = 0;
	int refWhiteLeft, refWhiteRight, refBlackFrontLeft, refBlackFrontRight = 0;
	int velArray[2]{0};
	int maxWhiteLeftRight, maxBlackLeftRight = 0;
	int currentVals[2]{0};
	int desVals[2]{0};
	int newVals[2]{0};
	int lastVals[2]{0};
	float faktor = 0;
    //for (uint8_t led = 0; led < 8; ++led) {
	//	global.robot.setLightColor(led, Color(Color::BLACK));
    //}
    running = false;

	
	// * LOOP
	 

	while (!this->shouldTerminate())
	{
        
         // * read accelerometer z-value
         
        accel_z = global.lis331dlh.getAccelerationForce(LIS331DLH::AXIS_Z);

        
        // * evaluate the accelerometer
         

        if (accel_z < -900 ) { //-0.9g
		this->sleep(MS2ST(3000));
		for (int i = 0; i < 4; i++) {
			for(size_t j = 0; j < 1000; j++){
				foo = global.vcnl4020[i].getProximityScaledWoOffset();
				initialVals[i] += foo;
			}
			initialVals[i] = round(initialVals[i]/1000);
			chprintf((BaseSequentialStream*) &SD1, "%04d\n",
			initialVals[i]);
		}
		refWhiteLeft = initialVals[1];
		refWhiteRight = initialVals[2];
		refBlackFrontLeft = initialVals[0];
		refBlackFrontRight = initialVals[3];
		maxBlackLeftRight = (refBlackFrontLeft+refBlackFrontRight)/2;
		maxWhiteLeftRight = ((refWhiteLeft+refWhiteRight)/2)-maxBlackLeftRight;

  
					 
            if (running) {
                // stop the robot
                running = false;
                global.motorcontrol.setTargetRPM(0, 0);
            } else {
                //start the robot
                running = true;
                global.robot.setLightColor(3, Color(Color::BLACK));
			global.robot.setLightColor(4, Color(Color::BLACK));
			global.robot.setLightColor(2, Color(Color::BLACK));
			global.robot.setLightColor(5, Color(Color::BLACK));
            }

        }
        if (running) {
			desVals[0] = 5;
			desVals[1] = 5;
            // Read the proximity values
			currentVals[0] = global.vcnl4020[0].getProximityScaledWoOffset();
			currentVals[1] = global.vcnl4020[3].getProximityScaledWoOffset();
			/*
			if ((currentVals[0] >= currentVals[1]) && ((currentVals[0]-lastVals[0]) >= 100)) {
				faktor = maxWhiteLeftRight/(currentVals[0]-maxBlackLeftRight);
				newVals[0] = round(desVals[0] + (faktor*desVals[0]));
				newVals[1] = desVals[1];
			} else if((currentVals[1] > currentVals[0]) && ((currentVals[1]-lastVals[1]) >= 100)) {
				faktor = maxWhiteLeftRight/(currentVals[1]-maxBlackLeftRight);
				newVals[1] = round(desVals[1] + (faktor*desVals[1]));
				newVals[0] = desVals[0];
			}
			*/
			//for(size_t i = 0; i < 2; i++){
			//	faktor = diffWhiteLeftRight/(currentVals[i]-diffBlackLeftRight);
			//	newVals[i] = round(desVals[i] + (faktor*desVals[i]));
			//}
			
			//setRpmSpeed(newVals);
            //lineFollowing(vcnl4020Proximity, rpmFuzzyCtrl);
            //setRpmSpeed(rpmFuzzyCtrl);
			//Regler();
            //types::position pos = global.robot.getOdometry();
            //chprintf((BaseSequentialStream*) &SD1, "%04d %04d %04d \n", pos.x, pos.y, pos.f_z);
            //int drehung[3] = {global.robot.getGyroscopeValue(0), global.robot.getGyroscopeValue(1), global.robot.getGyroscopeValue(2)};
            //chprintf((BaseSequentialStream*) &SD1, "%04d %04d %04d \n", drehung[0], drehung[1], drehung[2]);
			
			for(size_t i = 0; i < 2; i++){
				lastVals[i] = newVals[i];
			}
			int braitenberg = 2;
			if(braitenberg == 2){
				
				newVals[0] = static_cast<int>(maxSpeed * (((refWhiteLeft-(currentVals[1]-refBlackFrontLeft))/static_cast<float>(refWhiteLeft))));
				newVals[1] = static_cast<int>(maxSpeed * ((currentVals[0]/static_cast<float>(refWhiteLeft))));
				//newVals[0] = (newVals[0] + lastVals[0])/2;
				//newVals[1] = (newVals[1] + lastVals[1])/2;
				chprintf((BaseSequentialStream*) &SD1, "%04d   %04d \n", currentVals[0], currentVals[1]);
				if (newVals[0] == newVals[1] && newVals[0] == 0 ){
					newVals[0] = 100;
					newVals[1] = 0;
				}
				//else if (newVals[0] > maxSpeed - 10 && newVals[0] > maxSpeed -10){
					//newVals[0] = static_cast<int>(newVals[0] * 1.8);
					//newVals[1] = static_cast<int>(newVals[1] * 1.8);
				//}
				setRpmSpeed(newVals);

			}
			
			// plot y = (30000 - (x/300)^3)/30000, y = -((30000 - (x/300)^3))/30000) from -5000 to 12000 
			

        }

		this->sleep(CAN::UPDATE_PERIOD);
	}

  return RDY_OK;
}

