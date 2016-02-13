/*
 * Shooter.c
 *
 *  Created on: Feb 7, 2016
 *      Author: Erik
 */

#include "main.h"

Shooter initShooter(IncrementalController controller, PantherMotor motor1, PantherMotor motor2, int defaultSpeed)
{
	IncrementalController *newController = &controller;

	runIncrementalController(newController, 0.0);

	Shooter newShooter = {motor1, motor2, 0, defaultSpeed, 0, millis(), newController, millis(), defaultSpeed};
	return newShooter;
}

void turnShooterOn(Shooter *shooter)
{
	(*shooter).turnedOn = 1;
	(*shooter).SP = (*shooter).speed;

	forceOutput((*shooter).controller, (double) (*shooter).SP);

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
			speed = (*shooter).lastSpeed - 2;
			(*shooter).lastSpeed = speed;
			(*shooter).lastChangeTime = millis();
		}
		else
		{
			speed = (*shooter).lastSpeed;
		}

		speed = limit(speed, 400, 0);
	}

	(*shooter).SP = speed;

	runShooterAtSpeed(shooter);
}

void updateShooter(Shooter *shooter)
{
	if(absDouble((*(*shooter).controller).error > 5))
	{
		(*shooter).lastOffTime = millis();
	}
}

int isShooterUpToSpeed(Shooter *shooter)
{
	int atSpeed = (((millis() - (*shooter).lastOffTime)) > 100);

	lcdSetBacklight(uart1, atSpeed);

	return atSpeed;
}

void runShooterAtSpeed(Shooter *shooter)
{
	int speed = runIncrementalController(((*shooter).controller), (*shooter).SP);

	if(speed < 15)
		speed = 0;

	lcdPrint(uart1, 1, "Err: %f", (*(*shooter).controller).error);
	lcdPrint(uart1, 2, "Out: %d", speed);

	setPantherMotor((*shooter).motor1, speed);
	setPantherMotor((*shooter).motor2, speed);
}
