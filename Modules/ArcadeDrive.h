#ifndef ARCADEDRIVE_H_
#define ARCADEDRIVE_H_

#include "WPILib.h"
#include "../Defines.h"
#include "Utilities.h"
#include "../Globals.h"

class ArcadeDrive {
private:
	Victor *frontLeftMotor;
	Victor *frontRightMotor;
	Victor *backLeftMotor;
	Victor *backRightMotor;	
	
	Notifier *gyroNotifier;	
	
	AnalogChannel *magneticEncoder;
	
	//Servo *camServo;
	
	float driveY;
	float driveX;
	float throttle;
	float angle;
	static const float kp = 0.04;
	float setAngle;
	
	bool b_EnableDrive;
	
	void CheckIfDriveEnabled(Joystick* arcadeStick);
	
public:
	ArcadeDrive(void);
	~ArcadeDrive(void);
	
	void Drive(Joystick *arcadeStick);
	void AutonDrive();
	void AutonDriveBackward();
	void StopDrive();
	void SetUpDrive();
	//void RunGyro();
	//static void RunGyroDigital(void *owner);
	//void StartNotifier();
	//void StopNotifier();
	//float currentAngle;
	//double filteredAngle;
};


#endif
