/*
 * Shooter.c
 *
 *  Created on: Feb 7, 2016
 *      Author: Erik
 */

#include "main.h"

Shooter initShooter(PIDController controller, PantherMotor motor1, PantherMotor motor2, PantherMotor motor3, int fullCourtSpeed, int halfCourtSpeed, RedEncoder encoder)
{
	PIDController newController = controller;

	newController.setPoint = 0;

	Shooter newShooter = {motor1, motor2, motor3, 1, fullCourtSpeed, 0, millis(), newController, 0, millis(), encoder, fullCourtSpeed, SHOOTER_FULL_COURT, fullCourtSpeed, halfCourtSpeed};
	return newShooter;
}

void turnShooterOn(Shooter *shooter)
{
	(*shooter).turnedOn = 1;
	(*shooter).SP = (*shooter).speed;
	puts("Shooter turned on.");
}

void turnShooterOff(Shooter *shooter)
{
	(*shooter).turnedOn = 0;
	(*shooter).lastChangeTime = millis();
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
	(*shooter).speed += amount;

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
		(*shooter).SP = (*shooter).speed;
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
	setkP(&((*shooter).controller), kP);
}

void shooterSetKI(Shooter *shooter, double kI)
{
	setkI(&((*shooter).controller), kI);
}

void shooterSetKD(Shooter *shooter, double kD)
{
	setkD(&((*shooter).controller), kD);
}

void shooterSetKF(Shooter *shooter, double kF)
{
	setkF(&((*shooter).controller), kF);
}

void shooterSetErrorEpsilon(Shooter *shooter, int errorEpsilon)
{
	setErrorEpsilon(&((*shooter).controller), errorEpsilon);
}

void updateShooter(Shooter *shooter)
{
	(*shooter).processVariable = (int) getRedEncoderVelocity((*shooter).encoder);

	//lcdPrint(uart1, 1, "Speed: %d", (*shooter).processVariable);
	//lcdPrint(uart1, 1, "SP: %d", (*shooter).SP); TODO add back

	lcdPrint(uart1, 2, "Error: %d", (*shooter).controller.setPoint - (*shooter).processVariable);

	if(abs((int) (*shooter).controller.setPoint - (*shooter).processVariable) > 1)
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
		speed = (*shooter).SP * 0.37;
	}
	else
	{
		(*shooter).controller.setPoint = (*shooter).SP;
		speed = runPIDController(&((*shooter).controller),
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
		(*shooter).speed = (*shooter).halfCourtSpeed;
		break;

	case(SHOOTER_FULL_COURT):
		(*shooter).speed = (*shooter).fullCourtSpeed;
		break;
	}

	if((*shooter).turnedOn == 1)
		turnShooterOn(shooter);
}
