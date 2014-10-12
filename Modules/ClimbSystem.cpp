#include "ClimbSystem.h"
#include "../Defines.h"
#include "../Misc/DashboardConnecter.h"


ClimbSystem::ClimbSystem(ArcadeDrive* drivTrain){
	switchesHit = 0;
	dTrain = drivTrain;
	climbMotor1 = new Victor(CLIMB_MOTOR1);
	climbMotor2 = new Victor(CLIMB_MOTOR2);
	rotateMotor = new Victor(ROTATE_MOTOR);	
	slideMotor = new Relay(SLIDE_MOTOR);
	
	rightSlidePot = new AnalogChannel(RIGHT_SLIDE_POT);
	rightSlidePot->SetAverageBits(4);
	rotatePot = new AnalogChannel(ROTATE_POT);

	rightBackLimitSwitch = new DigitalInput(RIGHT_REAR_LIMIT_SWITCH);//0 is hooked
	rightFrontLimitSwitch = new DigitalInput(RIGHT_FRONT_LIMIT_SWITCH);
	
	topRodLimitSwitch = new DigitalInput(TOP_LIFT_LIMIT_SWITCH);
	botRodLimitSwitch = new DigitalInput(BOT_LIFT_LIMIT_SWITCH);
	leftSlideLimitSwitch = new DigitalInput(LEFT_SLIDE_LIMIT_SWITCH);
	//rotaterLimitSwitch = new DigitalInput(ROTATER_LIMIT_SWITCH);
	topHookLimitSwitch = new DigitalInput(TOP_HOOK_LIMIT_SWITCH);
	botHookLimitSwitch = new DigitalInput(BOTTOM_HOOK_LIMIT_SWITCH);
	
	driveEncoder = new Encoder(6, 7, true, Encoder::k1X);
	driveEncoder->SetDistancePerPulse(1.0);
	//driveEncoder->SetPIDSourceParameter(Encoder::kDistance);
	
	rotatePID = new AfterPID(-.015 * 1.5, 0, 0);
	lifterPID = new AfterPID(.00005555 * 18 * 1.5, 0, 0);
	
	driveEncoder->Start();
	
	//dash = new DashboardConnecter();
	
	rotateSetpoint = 0.0f;
	
	climbStateMachine = kFirstLevel;
	autonMachine = kReset;
	multiClimbMachine = kResetTel;
}
ClimbSystem::~ClimbSystem(void){
	
}

int ClimbSystem::CheckSafety()
{
	int numFailures = 0;
	if((topRodLimitSwitch->Get() == 1 && leftJoy < 0) || (botRodLimitSwitch->Get() == 1 && leftJoy > 0))
	{
		leftJoy = 0;
		driveEncoder->Reset();
		numFailures++;
	}
	if((rotatePot->GetAverageValue() > ROTATE_POT_MAX && twistJoy < 0) || (rotatePot->GetAverageValue() < ROTATE_POT_MIN && twistJoy > 0))
	{
		twistJoy = 0;
		numFailures++;
	}
	if(slideForward == 1 && (rightFrontLimitSwitch->Get() == 1))//servo 8, set 0
	{
		slideForward = 0;
		numFailures++;
	}
	if(slideBackward == 1 && rightBackLimitSwitch->Get() == 1)
	{
		slideBackward = 0;
		numFailures++;
	}
	return numFailures;
}

void ClimbSystem::RunClimbSystem(Joystick *userJoy)
{	
	printf("Distance: %f\n", driveEncoder->GetDistance());
	

	if(rightSlidePot->GetAverageVoltage() <= 0.0)
	{
		printf("Pot is unplugged, check connection\n");
	}
	
	if(userJoy->GetRawButton(2))
	{
		Climb(userJoy);
	}
	else
	{	
		// Grab axes and save them
		leftJoy = userJoy->GetY() * -1.0f;
		twistJoy = userJoy->GetTwist() * -1.0f;					
		slideForward = userJoy->GetRawButton(6);
		slideBackward = userJoy->GetRawButton(5);
		
		// Deadbands
		leftJoy = Utilities::deadbandValue(leftJoy, 0.125);
		twistJoy = Utilities::deadbandValue(twistJoy, 0.25);		
		
		CheckSafety(); // Check if we are within our limits
		//printf("Switch: %d\t LeftJoy: %f\n", topRodLimitSwitch->Get(), leftJoy);
		
		// Run motors
		climbMotor1->Set(leftJoy);
		climbMotor2->Set(leftJoy);
		rotateMotor->Set(twistJoy);
	
		if(slideForward == 1)//servo 8, set 0
			slideMotor->Set(Relay::kReverse);//forward
		else if(slideBackward == 1)
			slideMotor->Set(Relay::kForward);
		else
			slideMotor->Set(Relay::kOff);
	}
#ifdef __CLIMB_DEBUG	
	printf("Distance: %f\t\tRotate Pot: %d\n", 
			driveEncoder->GetDistance(), rotatePot->GetAverageValue());
#endif // __CLIMB_DEBUG
}

void ClimbSystem::Climb(Joystick *gamePad){
	
	switch(climbStateMachine){
	case kFirstLevel:
		MultiClimb();
		break;
	case kSecondLevel:
		MultiClimb();
		break;
	case kThirdLevel:
		MultiClimb();
		break;
	case kFinalPosition:
		break;
	}	
}

void ClimbSystem::ResetAuton()
{
	autonMachine = kReset;
	return;
}

bool ClimbSystem::AutonSetup(Joystick *userJoy, bool bTrueAuton)
{
	bool stateDone = false;
	float lifterOutput, rotatorOutput;
	
	static int counter = 0;
	
	if(bTrueAuton)
	{
		MultiClimb();
		return false;
	}
	
	switch(autonMachine)
	{
	case kReset:
		lifterOutput = .75;
		rotatorOutput = 0;
		
		if(botRodLimitSwitch->Get() == 1)
		{
			rotatePID->ResetPID();
			lifterPID->ResetPID();
			
			driveEncoder->Reset();
			autonMachine = kStepOne;
		}
		break;
	case kStepOne:
		lifterOutput = lifterPID->GetOutput(driveEncoder->GetDistance(), autonVals[0][0], 100, true);
		rotatorOutput = rotatePID->GetOutput(rotatePot->GetAverageValue(), autonVals[0][1], 20, true);
		
		if(lifterOutput == 0.0f && rotatorOutput == 0.0f)
		{
			counter++;
			if(counter >= 250)
			{
				autonMachine = kStepTwo;
				counter = 0;
			}
		}
		else
		{
			counter = 0;
		}
		break;
	case kStepTwo:
		
		rotatePID->SetPID(-.015 * 2.5, 0, 0,1);
		lifterPID->SetPID(.00005555 * 18 * 2, 0, 0,1);
				
		lifterOutput = lifterPID->GetOutput(driveEncoder->GetDistance(), autonVals[1][0], 100, true);
		rotatorOutput = rotatePID->GetOutput(rotatePot->GetAverageValue(), autonVals[1][1], 10, true);
		
		if(lifterOutput == 0.0f && rotatorOutput == 0.0f)
		{
			autonMachine = kStepThree;
		}
		break;
	case kStepThree:
		
		rotatePID->SetPID(-.015 * 2, 0, 0,1);
		lifterPID->SetPID(.00005555 * 18 * 2, 0, 0,1);

		lifterOutput = lifterPID->GetOutput(driveEncoder->GetDistance(), autonVals[2][0], 100, true);
		rotatorOutput = rotatePID->GetOutput(rotatePot->GetAverageValue(), autonVals[2][1], 20, true);
		
		if(lifterOutput == 0.0f && rotatorOutput == 0.0f)
		{
			autonMachine = kStepFour;
		}
		break;
	case kStepFour:
		rotatePID->SetPID(-.015 * 2, 0, 0,1);	
		lifterPID->SetPID(.00005555 * 18 * 2, 0, 0,1);
		
		lifterOutput = lifterPID->GetOutput(driveEncoder->GetDistance(), autonVals[3][0], 100, true);
		rotatorOutput = rotatePID->GetOutput(rotatePot->GetAverageValue(), autonVals[3][1], 10, true);
		
		if(lifterOutput == 0.0f && rotatorOutput == 0.0f)
		{
			//autonMachine = kMoveForward;
		}
		break;
	case kMoveForward:
		counter++;
		if(counter >= 300)
		{
			dTrain->StopDrive();
			driveEncoder->Reset();
			counter = 0;
			autonMachine = kExtendOne;
		}
		else
		{
			dTrain->AutonDrive();
		}
		break;
	case kExtendOne:
		lifterOutput = lifterPID->GetOutput(driveEncoder->GetDistance(), autonVals[4][0], 100, true);
		rotatorOutput = rotatePID->GetOutput(rotatePot->GetAverageValue(), autonVals[4][1], 10, true);
		
		if(lifterOutput == 0.0f && rotatorOutput == 0.0f)
		{
			autonMachine = kExtendTwo;
		}
		break;
	case kExtendTwo:

		lifterOutput = lifterPID->GetOutput(driveEncoder->GetDistance(), autonVals[5][0], 100, true);
		rotatorOutput = rotatePID->GetOutput(rotatePot->GetAverageValue(), autonVals[5][1], 10, true);
		
		if(lifterOutput == 0.0f && rotatorOutput == 0.0f)
		{
			autonMachine = kMoveBackward;
		}
		break;
			break;
	case kMoveBackward:
			if(botHookLimitSwitch->Get() == 1)
			{
				dTrain->StopDrive();
				counter = 0;
				autonMachine = kResetTwo;
			}
			else
			{
				dTrain->AutonDriveBackward();
			}
			break;
	case kResetTwo:
			lifterOutput = .75;
			rotatorOutput = 0;
			
			if(botRodLimitSwitch->Get() == 1)
			{
				rotatePID->ResetPID();
				lifterPID->ResetPID();
				
				driveEncoder->Reset();
				autonMachine = kDone;
			}
			break;
	case kDone:
		break;
	}
	
	leftJoy = lifterOutput;
	twistJoy = rotatorOutput;
	
	printf("Before: Left Joy: %f\t\t Twist Joy: %f\t\tSafety: %d\n", leftJoy, twistJoy, CheckSafety());
	
	printf("Current State:%d\n", autonMachine);
	printf("After: Left Joy: %f\t\t Twist Joy: %f\n", leftJoy, twistJoy);
	climbMotor1->Set(leftJoy);
	climbMotor2->Set(leftJoy);
	rotateMotor->Set(twistJoy);
	
	return stateDone;
}

bool ClimbSystem::MultiClimb(){
	bool IsDone;
	float lifterOutput = 0;
	float rotatorOutput = 0;
	
	switch(multiClimbMachine)
	{
	case kResetTel:
			lifterOutput = 0.75;
			rotatorOutput = 0;
			
			printf("Safety: %d\n", botRodLimitSwitch->Get());
			if(botRodLimitSwitch->Get() == 1){
				switchesHit++;
			}
			if(botRodLimitSwitch->Get() == 1 && switchesHit > 500)
			{
				printf("Switch hit\n");
				driveEncoder->Reset();
				multiClimbMachine = kLowerLifter;
			}
			break;
	case kLowerLifter:
		lifterOutput = -1;
		if(climbStateMachine == kFirstLevel)
		{
			if(driveEncoder->GetDistance() < -34750)
			{
				lifterOutput = 0;
				multiClimbMachine = kLockSlide;
			}
		}
		if(climbStateMachine == kSecondLevel)
		{
			if((driveEncoder->GetDistance()) < -9000)
			{
				lifterOutput = 0;
				//multiClimbMachine = kLockSlide;
			}
			slideForward = 1;
			if(rightFrontLimitSwitch->Get() == 1)
			{
				multiClimbMachine = kLockSlide;
				driveEncoder->Reset();
			}
		}
		break;
	case kLockSlide:
		if(climbStateMachine == kFirstLevel)
		{
			slideForward = 1;
			if(leftSlideLimitSwitch->Get() == 1)
			{
				slideForward = 0;
				driveEncoder->Reset();
				multiClimbMachine = kPositionLifter;
			}
		}
		if(climbStateMachine == kSecondLevel)
		{
			lifterOutput = -1;
			if(driveEncoder->GetDistance() < -27000)
			{
				lifterOutput = 0;
				slideBackward = 1;
			}
			if(rightBackLimitSwitch->Get() == 1)
			{
				lifterOutput = -1;
				if((driveEncoder->GetDistance()) < -78000)
				{
					lifterOutput = 0;
					climbStateMachine = kFirstLevel;
				}
			}
		}
		break;
	case kPositionLifter:
		lifterOutput = 1;
		if(driveEncoder->GetDistance() > 4000)
		{
			rotatorOutput = -.75;
			CheckSafety();
		}
		if(botRodLimitSwitch->Get() == 1)
		{
			lifterOutput = 0;
			rotatorOutput = .75;
			if(topHookLimitSwitch->Get() == 1)
			{
				rotatorOutput = 0;
				if(climbStateMachine == kFirstLevel)
				{
					driveEncoder->Reset();
					climbStateMachine = kSecondLevel;
				}
				else if((climbStateMachine == kSecondLevel))
				{
					climbStateMachine = kThirdLevel;
				}
				multiClimbMachine = kLowerLifter;
			}
		}
		break;
	}
	leftJoy = lifterOutput;
	twistJoy = rotatorOutput;
	
	/*printf("Before: Left Joy: %f\t\t Twist Joy: %f\t\tSafety: %d\n", leftJoy, twistJoy, CheckSafety());
	printf("After: Left Joy: %f\t\t Twist Joy: %f\n", leftJoy, twistJoy);*/
	CheckSafety();
	
	climbMotor1->Set(leftJoy);
	climbMotor2->Set(leftJoy);
	
	rotateMotor->Set(twistJoy);
	
	if(slideForward == 1)//servo 8, set 0
			slideMotor->Set(Relay::kReverse);//forward
		else if(slideBackward == 1)
			slideMotor->Set(Relay::kForward);
		else
			slideMotor->Set(Relay::kOff);
	return IsDone;
}
