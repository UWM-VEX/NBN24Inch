/*
 * Shooter.h
 *
 *  Created on: Feb 7, 2016
 *      Author: Erik
 */

#ifndef INCLUDE_SHOOTER_H_
#define INCLUDE_SHOOTER_H_

#define SHOOTER_FULL_COURT 0
#define SHOOTER_HALF_COURT 1

struct Shooter{

	PantherMotor motor1;
	PantherMotor motor2;
	PantherMotor motor3;
	int turnedOn;
	int SP;
	int lastSpeed;
	long lastChangeTime;
	PIDController controller;
	int processVariable;
	long lastOffTime;
	RedEncoder *encoder;
	int speed;
	int shooterMode;
	int fullCourtSpeed;
	int halfCourtSpeed;

}typedef Shooter;

Shooter initShooter(PIDController controller, PantherMotor motor1, PantherMotor motor2, PantherMotor motor3, int fullCourtSpeed, int halfCourtSpeed, RedEncoder encoder);
void turnShooterOn(Shooter *shooter);
void turnShooterOff(Shooter *shooter);
void changeShooterSP(Shooter *shooter, int SP);
void incrementShooterSP(Shooter *shooter, int amount);
void runShooter(Shooter *shooter);
void shooterSetKP(Shooter *shooter, double kP);
void shooterSetKI(Shooter *shooter, double kI);
void shooterSetKD(Shooter *shooter, double kD);
void shooterSetKF(Shooter *shooter, double kF);
void shooterSetErrorEpsilon(Shooter *shooter, int errorEpsilon);
void updateShooter(Shooter *shooter);
int isShooterUpToSpeed(Shooter *shooter);
void runShooterAtSpeed(Shooter *shooter);
void shootFullCourt(Shooter *shooter);
void shootHalfCourt(Shooter *shooter);
void changeShooterMode(Shooter *shooter, int shooterMode);

#endif /* INCLUDE_SHOOTER_H_ */
