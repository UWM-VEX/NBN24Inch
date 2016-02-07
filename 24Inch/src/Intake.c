/*
 * Intake.c
 *
 *  Created on: Oct 29, 2015
 *      Author: Erik
 */

#include "main.h"

Intake initIntake(PantherMotor intake1Left, PantherMotor intake1Right, PantherMotor intake2, PantherMotor intake3)
{
	Intake newIntake = {intake1Left, intake1Right, intake2, intake3};
	return newIntake;
}

void intake1In(Intake intake)
{
	setPantherMotor(intake.intake1Left, 127);
	setPantherMotor(intake.intake1Right, 127);
	setPantherMotor(intake.intake2, 127);
}

void intake1Out(Intake intake)
{
	setPantherMotor(intake.intake1Left, -127);
		setPantherMotor(intake.intake1Right, -127);
		setPantherMotor(intake.intake2, -127);
}

void intake1Stop(Intake intake)
{
	setPantherMotor(intake.intake1Left, 0);
	setPantherMotor(intake.intake1Right, 0);
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
