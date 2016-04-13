#include "main.h"

/**
 * Initializes a new drive object composed of four PantherMotors, two encoders
 * and a gyro.
 */
Drive initDrive(PantherMotor frontLeftMotor, PantherMotor frontRightMotor,
		PantherMotor rearLeftMotor, PantherMotor rearRightMotor,
		Encoder leftEncoder, Encoder rightEncoder, Gyro gyro, int leftLineTracker,
		int rightLineTracker)
{
	Drive newDrive = {frontLeftMotor, frontRightMotor,
			rearLeftMotor, rearRightMotor, leftEncoder, rightEncoder, gyro,
			leftLineTracker, rightLineTracker};

	return newDrive;
}

/**
 * Assigns the left speed of the drive to left and the right speed of the drive
 * to right.
 */
void tankDrive(Drive drive, int left, int right)
{
	int leftSpeed = limit(left, 127, -127);
	int rightSpeed = limit(right, 127, -127);

	setPantherMotor(drive.frontLeftMotor, leftSpeed);
	setPantherMotor(drive.rearLeftMotor, leftSpeed);
	setPantherMotor(drive.frontRightMotor, rightSpeed);
	setPantherMotor(drive.rearRightMotor, rightSpeed);
}

/**
 * Drives the robot with the forward/backward speed given by magnitude and
 * the rotational speed given by rotation.
 */
void arcadeDrive(Drive drive, int magnitude, int rotation)
{
	int left = limit(magnitude + rotation, 127, -127);
	int right = limit(magnitude - rotation, 127, -127);

	tankDrive(drive, left, right);
}
