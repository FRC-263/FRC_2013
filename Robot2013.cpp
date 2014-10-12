//2013 Robot Code
//By:Tejas Prasad, Pranav Sathy
//Assisted by:Matt Krass
//Robot Name:Paarthurnax

//CHECK OPERATOR CONTROL
//Removed Camera Control Code
//Removed Gyro Code and Gyro Notifier
//Removed kAuton enum, EncoderA and EncoderB

#include "Modules/ArcadeDrive.h"
#include "Modules/Utilities.h"
#include "Modules/ClimbSystem.h"
#include "WPILib.h"
#include "Defines.h"
#include "Globals.h"

ArcadeDrive *driveTrain;
ClimbSystem *climb;
Joystick *driveStick;
Joystick *gamePad;
AnalogChannel *autonState;

class Robot2013 : public SimpleRobot
{
public:
	Robot2013(void)
	{
		GetWatchdog().SetExpiration(1);
		GetWatchdog().SetEnabled(false);
	
		driveTrain = new ArcadeDrive();		
		climb = new ClimbSystem(driveTrain);		
		
		driveStick = new Joystick(1);
		gamePad = new Joystick(2);
		autonState = new AnalogChannel(7);
	}
	
	void Autonomous(void)
	{		
		while(IsAutonomous())
		{
			GetWatchdog().SetEnabled(false);
			/*if(autonState->GetValue() > 500){
				climb->AutonSetup(gamePad, true);
			}*/
		}
	}

	void OperatorControl(void)
	{		
		while (IsOperatorControl())
		{
			driveTrain->Drive(driveStick);
			climb->RunClimbSystem(gamePad);
			if(gamePad->GetRawButton(9)){
				climb->ResetAuton();
			}
			if(gamePad->GetRawButton(8)){
				climb->AutonSetup(gamePad, false);
			}
#if 0
			printf("Joystick test data:\n\tX: %+1.2f\n\tY: %+1.2f\n\tZ: %+1.2f\n\tT: %+1.2f\n\t11: %d\n\t12: %d\n\t13: %d\n\t14: %d\n\t5: %d\n\t6: %d\n---------------------------------------------",
					gamePad->GetX(),
					gamePad->GetY(),
					gamePad->GetZ(),
					gamePad->GetTwist(),
					gamePad->GetRawButton(11),
					gamePad->GetRawButton(12),
					gamePad->GetRawButton(13),
					gamePad->GetRawButton(14),
					gamePad->GetRawButton(5),
					gamePad->GetRawButton(6));
#endif
		}
	}
};

START_ROBOT_CLASS_FRC(Robot2013);
