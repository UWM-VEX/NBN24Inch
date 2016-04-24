/*
 * Shooter.c
 *
 *  Created on: Feb 7, 2016
 *      Author: Erik
 */

#include "main.h"

Shooter initShooter(PIDController controller, PantherMotor motor1, PantherMotor motor2, PantherMotor motor3, PantherMotor spinner, int fullCourtSpeed, int halfCourtSpeed, RedEncoder encoder)
{
	PIDController newController = controller;

	newController.setPoint = 0;

	Shooter newShooter = {.motor1 = motor1, .motor2 = motor2, .motor3 = motor3, .spinner = spinner, .turnedOn = 1, .SP = fullCourtSpeed, .lastSpeed = 0, .lastChangeTime = millis(), .controller = newController, .processVariable = 0, .lastOffTime = millis(), .encoder = encoder, .speedWhenOn = fullCourtSpeed, .shooterMode = SHOOTER_FULL_COURT, .fullCourtSpeed = fullCourtSpeed, .halfCourtSpeed = halfCourtSpeed};
	return newShooter;
}

void turnShooterOn(Shooter *shooter)
{
	(*shooter).turnedOn = 1;
	(*shooter).SP = (*shooter).speedWhenOn;
	//puts("Shooter turned on.");
	setPantherMotor(shooter->spinner, 127);
}

void turnShooterOff(Shooter *shooter)
{
	(*shooter).turnedOn = 0;
	(*shooter).lastChangeTime = millis();
	setPantherMotor(shooter->spinner, 0);
}

void changeShooterSP(Shooter *shooter, int SP)
{
	switch((*shooter).shooterMode)
		{
		case(SHOOTER_HALF_COURT):
				(*shooter).halfCourtSpeed = SP;
				break;

		case(SHOOTER_FULL_COURT): default:
				(*shooter).fullCourtSpeed = SP;
				break;
		}
}

void incrementShooterSP(Shooter *shooter, int amount)
{
	(*shooter).speedWhenOn += amount;

	switch((*shooter).shooterMode)
	{
	case(SHOOTER_HALF_COURT):
			(*shooter).halfCourtSpeed += amount;
			break;

	case(SHOOTER_FULL_COURT): default:
			(*shooter).fullCourtSpeed += amount;
			break;
	}
}

void runShooter(Shooter *shooter)
{
	int speed;

	switch((*shooter).shooterMode)
	{
	case(SHOOTER_HALF_COURT):
		(*shooter).SP = (*shooter).halfCourtSpeed;
		break;

	case(SHOOTER_FULL_COURT): default:
		(*shooter).SP = (*shooter).fullCourtSpeed;
		break;
	}

	if((*shooter).turnedOn)
	{
		(*shooter).SP = (*shooter).speedWhenOn;
		speed = (*shooter).SP;

		(*shooter).lastSpeed = (*shooter).SP;
	}
	else
	{
		int dT = (int) (millis() - (*shooter).lastChangeTime);

		if(dT > 50)
		{
			speed = (*shooter).lastSpeed - 10;
			(*shooter).lastSpeed = speed;
			(*shooter).lastChangeTime = millis();
		}
		else
		{
			speed = (*shooter).lastSpeed;
		}

		speed = limit(speed, 500, 0);
	}

	(*shooter).SP = speed;

	runShooterAtSpeed(shooter);
}

void shooterSetKP(Shooter *shooter, double kP)
{
	PIDsetkP(&((*shooter).controller), kP);
}

void shooterSetKI(Shooter *shooter, double kI)
{
	PIDsetkI(&((*shooter).controller), kI);
}

void shooterSetKD(Shooter *shooter, double kD)
{
	PIDsetkD(&((*shooter).controller), kD);
}

void shooterSetKF(Shooter *shooter, double kF)
{
	PIDsetkF(&((*shooter).controller), kF);
}

void shooterSetErrorEpsilon(Shooter *shooter, int errorEpsilon)
{
	PIDsetErrorEpsilon(&((*shooter).controller), errorEpsilon);
}

void updateShooter(Shooter *shooter)
{
	(*shooter).processVariable = (int) getRedEncoderVelocity((*shooter).encoder);
	lcdPrint(uart1, 2, "Speed: %d", (*shooter).processVariable);
	lcdPrint(uart1, 1, "SP: %d", (*shooter).SP);

	//lcdPrint(uart1, 2, "Error: %d", (*shooter).controller.setPoint - (*shooter).processVariable);

	if(abs((int) (*shooter).controller.setPoint - (*shooter).processVariable) > 20)
	{
		(*shooter).lastOffTime = millis();
	}
}

int isShooterUpToSpeed(Shooter *shooter)
{
	return (((millis() - (*shooter).lastOffTime)) > 100);
}

void runShooterAtSpeed(Shooter *shooter)
{
	int speed;

	if((*shooter).turnedOn && (*shooter).processVariable == 0)
	{
		//Open loop fall back if the encoder is not working correctly
		speed = (*shooter).SP * shooter->controller.kF;
	}
	else
	{
		(*shooter).controller.setPoint = (*shooter).SP;
		speed = PIDRunController(&((*shooter).controller),
				(*shooter).processVariable);
	}

	speed = limit(speed, 127, 0);

	setPantherMotor((*shooter).motor1, speed);
	setPantherMotor((*shooter).motor2, speed);
	setPantherMotor((*shooter).motor3, speed);
}

void shootFullCourt(Shooter *shooter)
{
	changeShooterMode(shooter, SHOOTER_FULL_COURT);
}

void shootHalfCourt(Shooter *shooter)
{
	changeShooterMode(shooter, SHOOTER_HALF_COURT);
}

void changeShooterMode(Shooter *shooter, int shooterMode)
{
	(*shooter).shooterMode = shooterMode;

	switch(shooterMode)
	{
	case(SHOOTER_HALF_COURT):
		(*shooter).speedWhenOn = (*shooter).halfCourtSpeed;
		break;

	case(SHOOTER_FULL_COURT):
		(*shooter).speedWhenOn = (*shooter).fullCourtSpeed;
		break;
	}

	if((*shooter).turnedOn == 1)
		turnShooterOn(shooter);
}
