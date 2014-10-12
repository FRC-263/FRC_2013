#ifndef DEFINES_H_
#define DEFINES_H_

	// Enable debug prints
	#undef		__DRIVE_DEBUG
	#define		__CLIMB_DEBUG
	// END debug prints

	//PWM
	#define FRONT_LEFT_MOTOR 6
	#define FRONT_RIGHT_MOTOR 1
	#define BACK_LEFT_MOTOR 5
	#define BACK_RIGHT_MOTOR 2

	#define CLIMB_MOTOR1 3
	#define CLIMB_MOTOR2 4
	#define ROTATE_MOTOR 7
	
	#define SLIDE_MOTOR 1

	#define JOYSTICK 1
	
	#define ROTATE_POT_MIN 203
	#define ROTATE_POT_MAX 510

	#define ROTATE_POT_DS 1

	#define ROTATE_DEADBAND 0.1
	//Analog
	#define RIGHT_SLIDE_POT 2
	#define ROTATE_POT 1

	//Digital
	#define RIGHT_REAR_LIMIT_SWITCH 4
	#define RIGHT_FRONT_LIMIT_SWITCH 5
	#define ENCODER_A 6
	#define ENCODER_B 7
	#define TOP_LIFT_LIMIT_SWITCH 8
	#define BOT_LIFT_LIMIT_SWITCH 9
	#define LEFT_SLIDE_LIMIT_SWITCH 10
	#define ROTATER_LIMIT_SWITCH 11
	#define TOP_HOOK_LIMIT_SWITCH 3
	#define BOTTOM_HOOK_LIMIT_SWITCH 1

	#define CAMERA_SERVO 8

const int autonVals[6][2] = {{0, 340}, {-18494, 340}, {-18494, 250}, {-21549, 235}, {1500, 235}, {1500, 464}};

#define START_ROBOT_CLASS_FRC(_ClassName_) \
	RobotBase *FRC_userClassFactory() \
	{ \
		return new _ClassName_(); \
	} \
	extern "C" { \
		INT32 FRC_UserProgram_StartupLibraryInit() \
		{ \
			RobotBase::startRobotTask((FUNCPTR)FRC_userClassFactory); \
			printf("%s\n", awesome); \
			return 0; \
		} \
	}

#endif
