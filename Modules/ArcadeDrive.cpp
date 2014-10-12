#include "ArcadeDrive.h"
#include "WPILib.h"
#include "../Defines.h"
#include "../Globals.h"
#include <math.h>

ArcadeDrive::ArcadeDrive(){
	frontLeftMotor = new Victor(1);
	frontRightMotor = new Victor(2);
	backLeftMotor = new Victor(4);
	backRightMotor = new Victor(3);	
	
	/*camServo = new Servo(CAMERA_SERVO);
	camServo->SetAngle(120);
	*/
	b_EnableDrive = true;
}

ArcadeDrive::~ArcadeDrive()
{
	
}

/*void ArcadeDrive::StartNotifier()
{
	gyroNotifier->StartPeriodic(0.01);
}*/

void ArcadeDrive::AutonDrive()
{
	frontLeftMotor->Set(-1.0f);
	backLeftMotor->Set(+1.0f);
	frontRightMotor->Set(-1.0f);
	backRightMotor->Set(+1.0f);
	return;
}

void ArcadeDrive::AutonDriveBackward()
{
	frontLeftMotor->Set(+0.5f);
	backLeftMotor->Set(-0.5f);
	frontRightMotor->Set(+0.5f);
	backRightMotor->Set(-0.5f);
	return;
}

void ArcadeDrive::StopDrive()
{
	frontLeftMotor->Set(0);
	backLeftMotor->Set(0);
	frontRightMotor->Set(0);
	backRightMotor->Set(0);
	return;
}

void ArcadeDrive::Drive(Joystick *arcadeStick)
{
	CheckIfDriveEnabled(arcadeStick);
	if(b_EnableDrive)
	{
		double leftMotorValue =	0.0f;
		double rightMotorValue = 0.0f;
		
		driveX = arcadeStick->GetTwist() * +1.0f;
		driveY = arcadeStick->GetY() * -1.0f;
		throttle = (arcadeStick->GetThrottle() * -0.5f) + 0.5f;
		
		driveX = Utilities::deadbandValue(driveX, 0.125);
		driveY = Utilities::deadbandValue(driveY, 0.125);			
		
		leftMotorValue = Utilities::boundValue((driveY + driveX) * throttle, -1.0f, 1.0f);
		rightMotorValue = Utilities::boundValue((driveY - driveX) * throttle, -1.0f, 1.0f);
		if(driveX > 0){
			frontLeftMotor->Set(leftMotorValue * -1.0f);
			backRightMotor->Set(rightMotorValue * +1.0f);
		}
		else if(driveX < 0){
			frontRightMotor->Set(rightMotorValue * +1.0f);
			backLeftMotor->Set(leftMotorValue * -1.0f);
		}
		frontLeftMotor->Set(leftMotorValue * -1.0f);
		backLeftMotor->Set(leftMotorValue * +1.0f);
		frontRightMotor->Set(rightMotorValue * -1.0f);
		backRightMotor->Set(rightMotorValue * +1.0f);
		
	}
#ifdef __DRIVE_DEBUG	
	printf("Left: %+2.2f\t\tRight: %+2.2f\t\t Throttle: %+2.2f\n", leftMotorValue, rightMotorValue, throttle);
#endif // __DRIVE_DEBUG	
}

/*void ArcadeDrive::RunGyro()
{
	static double prevAngle = 0.0;
	static bool bJumped = false;
	angle = gyroSensor->GetAngle();
	if(fabs(angle) < 0.1){
		angle = 0;
		gyroSensor->Reset();
	}

	if(fabs(angle) < 400)
	{
		filteredAngle = Utilities::lowPassFilterRealtime(angle, prevAngle, 100, 10); //lower 10 to make filter tighter
		if(fabs(filteredAngle) > 1000 && !bJumped)
		{
			bJumped = true;
			printf("Jump! values were angle = %3.3f and prevAngle = %3.3f\n", angle, prevAngle);
		}
		currentAngle = filteredAngle;
		prevAngle = filteredAngle;
	}
}*/

/*void ArcadeDrive::RunGyroDigital(void *owner)
{
	ArcadeDrive *drive = (ArcadeDrive*)owner;
	drive->RunGyro();
}*/

void ArcadeDrive::SetUpDrive()
{
	frontLeftMotor->SetSafetyEnabled(false);
	frontRightMotor->SetSafetyEnabled(false);
	backLeftMotor->SetSafetyEnabled(false);
	backRightMotor->SetSafetyEnabled(false);
}

void ArcadeDrive::CheckIfDriveEnabled(Joystick* arcadeStick)
{
	if(arcadeStick->GetRawButton(10))
		b_EnableDrive = true;
	else if(arcadeStick->GetRawButton(11))
		b_EnableDrive = false;
	else
		b_EnableDrive = true;
	
}
