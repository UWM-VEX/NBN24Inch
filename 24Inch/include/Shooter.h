/*
 * Shooter.h
 *
 *  Created on: Feb 7, 2016
 *      Author: Erik
 */

#ifndef INCLUDE_SHOOTER_H_
#define INCLUDE_SHOOTER_H_

struct Shooter{

	PantherMotor motor1;
	PantherMotor motor2;
	int turnedOn;
	int SP;
	int lastSpeed;
	long lastChangeTime;
	long lastOffTime;
	int speed;

}typedef Shooter;

Shooter initShooter(PantherMotor motor1, PantherMotor motor2, int defaultSpeed);
void turnShooterOn(Shooter *shooter);
void turnShooterOff(Shooter *shooter);
void changeShooterSP(Shooter *shooter, int SP);
void incrementShooterSP(Shooter *shooter, int amount);
void runShooter(Shooter *shooter);
void runShooterAtSpeed(Shooter *shooter);

#endif /* INCLUDE_SHOOTER_H_ */
