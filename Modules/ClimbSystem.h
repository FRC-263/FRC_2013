#ifndef CLIMB_SYSTEM_H_
#define CLIMB_SYSTEM_H_

#include "../Defines.h"
#include "../Globals.h"
#include "../Misc/AfterPID.h"
#include "../Misc/DashboardConnecter.h"
#include "ArcadeDrive.h"
#include "Utilities.h"
#include "WPILib.h"


class ClimbSystem{
private:
	Victor *climbMotor1;
	Victor *climbMotor2;
	Victor *rotateMotor;
	
	Relay *slideMotor;

	AnalogChannel *rightSlidePot;
	AnalogChannel *rotatePot;
	
	DigitalInput *topRodLimitSwitch;
	DigitalInput *botRodLimitSwitch;
	DigitalInput *leftSlideLimitSwitch;
	DigitalInput *rightBackLimitSwitch;
	DigitalInput *rightFrontLimitSwitch;
	//DigitalInput *rotaterLimitSwitch;
	DigitalInput *topHookLimitSwitch;
	DigitalInput *botHookLimitSwitch;
	
	Encoder *driveEncoder;
			
	AfterPID *rotatePID;
	AfterPID *lifterPID;
		
	float leftJoy;
	float twistJoy;
	int	slideForward;
	int	slideBackward;
	
	float rotateSetpoint;

	typedef enum {kFirstLevel, kSecondLevel, kThirdLevel, kFinalPosition} ClimbState;
	typedef enum {kReset, kStepOne, kStepTwo, kStepThree, kStepFour, kMoveForward, kExtendOne, kExtendTwo, kMoveBackward, kResetTwo, kDone} AutonState;
	typedef enum {kResetTel, kLowerLifter, kLockSlide, kPositionLifter} MultiClimbState;
	
	ClimbState climbStateMachine;
	AutonState autonMachine;
	MultiClimbState multiClimbMachine;
	
	int CheckSafety();
	
	bool MultiClimb();
	bool FinalClimb();
	bool FinalPosition();
	
	ArcadeDrive* dTrain;
	
	int switchesHit;

public:
	ClimbSystem(ArcadeDrive* drivTrain);
	~ClimbSystem(void);
	bool AutonSetup(Joystick *userJoy, bool bTrueAuton = true);
	
	void RunClimbSystem(Joystick *userJoy);
	void Climb(Joystick *gamePad);
	
	void ResetAuton();
};

#endif
