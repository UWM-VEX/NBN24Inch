/*
 * Shooter.c
 *
 *  Created on: Feb 7, 2016
 *      Author: Erik
 */

#include "main.h"

Shooter initShooter(PIDController controller, PantherMotor motor1, PantherMotor motor2, int defaultSpeed, RedEncoder encoder)
{
	PIDController newController = controller;

	newController.setPoint = 0;

	Shooter newShooter = {motor1, motor2, 0, defaultSpeed, 0, millis(), newController, 0, millis(), &encoder, defaultSpeed};
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
	(*shooter).speed = SP;
}

void incrementShooterSP(Shooter *shooter, int amount)
{
	(*shooter).speed += amount;
}

void runShooter(Shooter *shooter)
{
	int speed;

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

	lcdPrint(uart1, 1, "Speed: %d\n", (*shooter).processVariable);

	lcdPrint(uart1, 2, "Error: %d\n", (*shooter).controller.setPoint - (*shooter).processVariable);

	if(abs((int) (*shooter).controller.setPoint - (*shooter).processVariable) > 250)
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
	(*shooter).controller.setPoint = (*shooter).SP;
	int speed = runPIDController(&((*shooter).controller),
			(*shooter).processVariable);

	speed = limit(speed, 127, 0);

	setPantherMotor((*shooter).motor1, speed);
	setPantherMotor((*shooter).motor2, speed);
}
