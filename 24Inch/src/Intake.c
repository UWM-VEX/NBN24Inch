/*
 * Intake.c
 *
 *  Created on: Oct 29, 2015
 *      Author: Erik
 */

#include "main.h"

Intake initIntake(PantherMotor intake1, PantherMotor intake2, PantherMotor intake3)
{
	Intake newIntake = {intake1, intake2, intake3};
	return newIntake;
}

void intake1In(Intake intake)
{
	setPantherMotor(intake.intake1, 127);
	setPantherMotor(intake.intake2, 127);
}

void intake1Out(Intake intake)
{
	setPantherMotor(intake.intake1, -127);
		setPantherMotor(intake.intake2, -127);
}

void intake1Stop(Intake intake)
{
	setPantherMotor(intake.intake1, 0);
	setPantherMotor(intake.intake2, 0);
}

void intake2In(Intake intake)
{
	setPantherMotor(intake.intake3, 127);
}

void intake2Out(Intake intake)
{
	setPantherMotor(intake.intake3, -127);
}

void intake2Stop(Intake intake)
{
	setPantherMotor(intake.intake3, 0);
}
