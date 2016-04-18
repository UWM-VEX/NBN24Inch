/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Purdue University ACM SIG BOTS nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */

/**
 * This is where you declare all of the actions the robot will take.
 * The options are DriveForTime which is useful for driving into something
 * but shouldn't be used elsewhere, DriveToWayPoint, which will handle
 * driving forward and back, strafing, and turning (turning must be in
 * its own step) and AutoLiftToHeight which will bring the lift to a
 * specified height (Note: Once the step where this function is used has
 * completed, the lift will move down due to gravity. To avoid this,
 * create a new AutoLiftToHeight function to keep the lift at the desired
 * height. Also, the lift to height code isn't perfectly tuned yet,
 * if the autonomous stalls with the autoLiftToHeight function, help the
 * lift up.)
 *
 * Running the pickup or spinner does not require an object to be declared
 * or instantiated, an example is shown below.
 */

int isAuto = 1;

long int stepStartTime;

DriveToWP drive24Forward;
DriveToWP turn90Right;
DriveToWP turn90Left;
DriveToWP drive24Backward;

DriveToWP worlds1TurnToPile1;
DriveToWP worlds1DriveToPile1;
DriveToWP worlds1DriveBackToShoot1;
DriveToWP worlds1TurnBackToCorner1;
DriveToWP worlds1TurnToShoot1;
DriveToWP worlds1FirstTurnToPile2;
DriveToWP worlds1FirstDriveToToPile2;
DriveToWP worlds1SecondTurnToPile2;
DriveToWP worlds1SecondDriveToPile2;
DriveToWP worlds1BackAwayFromPile2;
DriveToWP worlds1TurnToShoot2;
DriveToWP worlds1TurnToPile3;
DriveToWP worlds1DriveToPile3;
DriveToWP worlds1TurnToShoot3;

DriveToWP worlds2TurnToPile1;
DriveToWP worlds2DriveToPile1;
DriveToWP worlds2StraightenOut1;
DriveToWP worlds2DriveBackToBase1;
DriveToWP worlds2TurnToShoot1;
DriveToWP worlds2FirstTurnToPile2;
DriveToWP worlds2FirstDriveToPile2;
DriveToWP worlds2SecondTurnToPile2;
DriveToWP worlds2SecondDriveToPile2;
DriveToWP worlds2TurnToShoot2;
DriveToWP worlds2FirstTurnToPile3;
DriveToWP worlds2FirstDriveToPile3;
DriveToWP worlds2SecondTurnToPile3;
DriveToWP worlds2SecondDriveToPile3;
DriveToWP worlds2BackAwayFromPile3;
DriveToWP worlds2TurnToShoot3;


/**
 * Runs at the start of autonomous. Steps should be initialized here.
 */
void autonomousInit()
{
	/**
	 * Here, the different steps are instantiated and details are
	 * given about them. By hovering over the function name, you can see a
	 * list of the arguments to pass in.
	 */

	drive24Forward = initDriveToWP(robotDrive, 24, 0);
	turn90Right = initDriveToWP(robotDrive, 0, 45);
	turn90Left = initDriveToWP(robotDrive, 0, -90);
	drive24Backward = initDriveToWP(robotDrive, -24, 0);

	if(autonomousSelection == WORLDS_1){
		worlds1TurnToPile1 = initDriveToWP(robotDrive, 0, -20);
		worlds1TurnBackToCorner1 = initDriveToWP(robotDrive, 0, 20);
		worlds1DriveToPile1 = initDriveToWP(robotDrive, 30, 0);
		worlds1DriveBackToShoot1 = initDriveToWP(robotDrive, -42, 0);
		worlds1TurnToShoot1 = initDriveToWP(robotDrive, 0, -41);
		worlds1FirstTurnToPile2 = initDriveToWP(robotDrive, 0, 45);
		worlds1FirstDriveToToPile2 = initDriveToWP(robotDrive, 48, 0);
		worlds1SecondTurnToPile2 = initDriveToWP(robotDrive, 0, 90);
		worlds1SecondDriveToPile2 = initDriveToWP(robotDrive, 12, 0);
		worlds1TurnToShoot2 = initDriveToWP(robotDrive, 0, -131);
		worlds1BackAwayFromPile2 = initDriveToWP(robotDrive, -48, 0);
		worlds1TurnToPile3 = initDriveToWP(robotDrive, 0, 180);
		worlds1DriveToPile3 = initDriveToWP(robotDrive, 24, 0);
		worlds1TurnToShoot3 = initDriveToWP(robotDrive, 0 , -180);
		driveToWPSetMaxSpeed(&worlds1DriveToPile1, 70);
		driveToWPSetMaxSpeed(&worlds1SecondDriveToPile2, 40);
	}
	else if(autonomousSelection == WORLDS_2){
		worlds2TurnToPile1 = initDriveToWP(robotDrive, 0, -20);
		worlds2DriveToPile1 = initDriveToWP(robotDrive, 30, 0);
		worlds2StraightenOut1 = initDriveToWP(robotDrive, 0, 20);
		worlds2DriveBackToBase1 = initDriveToWP(robotDrive, -41, 0);
		worlds2TurnToShoot1 = initDriveToWP(robotDrive, 0, -40);
		worlds2FirstTurnToPile2 = initDriveToWP(robotDrive, 0, 43);
		worlds2FirstDriveToPile2 = initDriveToWP(robotDrive, 66, 0);
		worlds2SecondTurnToPile2 = initDriveToWP(robotDrive, 0, -90);
		worlds2SecondDriveToPile2 = initDriveToWP(robotDrive, 18, 0);
		worlds2TurnToShoot2 = initDriveToWP(robotDrive, 0 , 30);
		worlds2FirstTurnToPile3 = initDriveToWP(robotDrive, 0, -120);
		worlds2FirstDriveToPile3 = initDriveToWP(robotDrive, 18, 0 );
		worlds2SecondTurnToPile3 = initDriveToWP(robotDrive, 0, -90);
		worlds2SecondDriveToPile3 = initDriveToWP(robotDrive, 36, 0);
		worlds2BackAwayFromPile3 = initDriveToWP(robotDrive, -48, 0);
		worlds2TurnToShoot3 = initDriveToWP(robotDrive, 0, -135);
		driveToWPSetMaxSpeed(&worlds2DriveToPile1, 70);
		driveToWPSetMaxSpeed(&worlds2SecondDriveToPile3, 40);

	}
	autonomousInfo.lastStep = 0;

	autonomousInfo.step = 1;
	autonomousInfo.isFinished = 0;

	delay(500);

	stepStartTime = millis();

}

/**
 * Runs continuously during autonomous, should exit relatively promptly.
 */
void autonomousPeriodic()
{
	if(autonomousInfo.step != autonomousInfo.lastStep)
	{
		stepStartTime = millis();
	}

	updateShooter(&robotShooter);
	runShooter(&robotShooter);

	autonomousInfo.elapsedTime = millis() - stepStartTime;

	lcdPrint(uart1, 2, "Step: %d", autonomousInfo.step);

	switch(autonomousSelection)
	{
	case(MODE_1):
		switch(autonomousInfo.step)
		{


		case(1):
				driveToWP(&drive24Forward);
				autonomousInfo.isFinished = drive24Forward.isFinished;
				break;

			case(2):
				driveToWP(&turn90Right);
				autonomousInfo.isFinished = turn90Right.isFinished;
				break;

			case (3):
				driveToWP(&turn90Left);
				autonomousInfo.isFinished = turn90Left.isFinished;
				break;

			case(4):
				driveToWP(&drive24Backward);
				autonomousInfo.isFinished = drive24Backward.isFinished;
				break;

			default:
				isAuto = 0;
				break;

				}
				break;

		case(JUST_TURN):
			switch(autonomousInfo.step)
			{
			case(1):
				driveToWP(&turn90Right);
				autonomousInfo.isFinished = turn90Right.isFinished;
				break;

			default:
				isAuto = 0;
			}
			break;

		case(JUST_DRIVE):
			switch(autonomousInfo.step)
			{
			case(1):
				driveToWP(&drive24Forward);
				autonomousInfo.isFinished = drive24Forward.isFinished;
				break;
			default:
				isAuto = 0;
				break;
			}
			break;

		case(WORLDS_1):
		switch(autonomousInfo.step)
		{
		case(1):
			turnShooterOn(&robotShooter);
			intake1In(robotIntake);
			driveToWP(&worlds1TurnToPile1);
			autonomousInfo.isFinished = worlds1TurnToPile1.isFinished || autonomousInfo.elapsedTime > 10000;
			break;

		case(2):
			driveToWP(&worlds1DriveToPile1);
			autonomousInfo.isFinished = worlds1DriveToPile1.isFinished;
			break;

		case(3):
			driveToWP(&worlds1TurnBackToCorner1);
			autonomousInfo.isFinished = worlds1TurnBackToCorner1.isFinished;
			break;

		case(4):
			driveToWP(&worlds1DriveBackToShoot1);
			autonomousInfo.isFinished = worlds1DriveBackToShoot1.isFinished;
			break;
		case(5):
			driveToWP(&worlds1TurnToShoot1);
			autonomousInfo.isFinished = worlds1TurnToShoot1.isFinished;
			intake2In(robotIntake);
			break;
		case(6):
			autonomousInfo.isFinished = autonomousInfo.elapsedTime > 4000;
			break;
		case(7):
			driveToWP(&worlds1FirstTurnToPile2);
			autonomousInfo.isFinished = worlds1FirstTurnToPile2.isFinished;
			shootHalfCourt(&robotShooter);
			break;
		case(8):
			driveToWP(&worlds1FirstDriveToToPile2);
			autonomousInfo.isFinished = worlds1FirstDriveToToPile2.isFinished;
			break;
		case(9):
			driveToWP(&worlds1SecondTurnToPile2);
			autonomousInfo.isFinished = worlds1SecondTurnToPile2.isFinished;
			break;
		case(10):
			driveToWP(&worlds1SecondDriveToPile2);
			intake2Stop(robotIntake);
			autonomousInfo.isFinished = worlds1SecondDriveToPile2.isFinished
					|| autonomousInfo.elapsedTime > 6000;
			break;
		case(11):
			driveToWP(&worlds1BackAwayFromPile2);
			intake2Stop(robotIntake);
			autonomousInfo.isFinished = worlds1BackAwayFromPile2.isFinished;
			break;
		case(12):
			driveToWP(&worlds1TurnToShoot2);
			intake2In(robotIntake);
			autonomousInfo.isFinished = worlds1TurnToShoot2.isFinished;
			break;
		case(13):
			autonomousInfo.isFinished = autonomousInfo.elapsedTime > 4000;
			break;
		case(14):
			driveToWP(&worlds1TurnToPile3);
			autonomousInfo.isFinished = worlds1TurnToPile3.isFinished;
			break;
		case(15):
			driveToWP(&worlds1DriveToPile3);
			autonomousInfo.isFinished = worlds1DriveToPile3.isFinished;
			break;
		case(16):
			driveToWP(&worlds1TurnToShoot3);
			autonomousInfo.isFinished = worlds1TurnToShoot3.isFinished;
			break;
		case(17):
			autonomousInfo.isFinished = autonomousInfo.elapsedTime > 4000;
			break;
		default:
			isAuto = 0;
			break;
		}
		break;

		case(WORLDS_2):
			switch(autonomousInfo.step){
				case(1):
					intake1In(robotIntake);
					shootFullCourt(&robotShooter);
					driveToWP(&worlds2TurnToPile1);
					autonomousInfo.isFinished = worlds2TurnToPile1.isFinished;
					break;
				case(2):
					driveToWP(&worlds2DriveToPile1);
					autonomousInfo.isFinished = worlds2DriveToPile1.isFinished;
					break;
				case(3):
					driveToWP(&worlds2StraightenOut1);
					autonomousInfo.isFinished = worlds2StraightenOut1.isFinished;
					break;
				case(4):
					driveToWP(&worlds2DriveBackToBase1);
					autonomousInfo.isFinished = worlds2DriveBackToBase1.isFinished;
					break;
				case(5):
					intake2In(robotIntake);
					driveToWP(&worlds2TurnToShoot1);
					autonomousInfo.isFinished = worlds2TurnToShoot1.isFinished;
					break;
				case(6):
					autonomousInfo.isFinished = autonomousInfo.elapsedTime > 4000;
					break;
				case(7):
					intake2Stop(robotIntake);
					driveToWP(&worlds2FirstTurnToPile2);
					autonomousInfo.isFinished = worlds2FirstTurnToPile2.isFinished;
					break;
				case(8):
					driveToWP(&worlds2FirstDriveToPile2);
					autonomousInfo.isFinished = worlds2FirstDriveToPile2.isFinished;
					break;
				case(9):
					driveToWP(&worlds2SecondTurnToPile2);
					autonomousInfo.isFinished = worlds2SecondTurnToPile2.isFinished;
					break;
				case(10):
					driveToWP(&worlds2SecondDriveToPile2);
					autonomousInfo.isFinished = worlds2SecondDriveToPile2.isFinished;
					shootHalfCourt(&robotShooter);
					break;
				case(11):
					driveToWP(&worlds2TurnToShoot2);
					intake2In(robotIntake);
					autonomousInfo.isFinished = worlds2TurnToShoot2.isFinished;
					break;
				case(12):
					autonomousInfo.isFinished = autonomousInfo.elapsedTime > 4000;
					break;
				case(13):
					driveToWP(&worlds2FirstTurnToPile3);
					autonomousInfo.isFinished = worlds2FirstTurnToPile3.isFinished;
					intake2Stop(robotIntake);
					break;
				case(14):
					driveToWP(&worlds2FirstDriveToPile3);
					autonomousInfo.isFinished = worlds2FirstDriveToPile3.isFinished;
					break;
				case(15):
					driveToWP(&worlds2SecondTurnToPile3);
					autonomousInfo.isFinished = worlds2SecondTurnToPile3.isFinished;
					break;
				case(16):
					driveToWP(&worlds2SecondDriveToPile3);
					autonomousInfo.isFinished = worlds2SecondDriveToPile3.isFinished;
					break;
				case(17):
					driveToWP(&worlds2BackAwayFromPile3);
					autonomousInfo.isFinished = worlds2BackAwayFromPile3.isFinished;
					intake2In(robotIntake);
					break;
				case(18):
					driveToWP(&worlds2TurnToShoot3);
					autonomousInfo.isFinished = worlds2TurnToShoot3.isFinished;
					break;
				case(19):
					autonomousInfo.isFinished = autonomousInfo.elapsedTime > 4000;
					break;
				default:
					isAuto = 0;
					break;
		}
		break;

		case(DO_NOTHING):
			isAuto = 0;
		break;

		default:
			isAuto = 0;
			break;

		break;

	}
	autonomousInfo.lastStep = autonomousInfo.step;

	if(autonomousInfo.isFinished)
	{
		autonomousInfo.step ++;
		autonomousInfo.isFinished = 0;
	}

}

void autonomous()
{
	lcdSetText(uart1, 1, "started");

	autonomousInit();

	//lcdSetText(uart1, 1, "initialized");

	while(isAuto)
	{
		autonomousPeriodic();

		if(isOnline())
		{
			if(!isAutonomous() || !isEnabled()) isAuto = 0;
		}

		delay(20);

		puts("Autonomous");
	}
}




