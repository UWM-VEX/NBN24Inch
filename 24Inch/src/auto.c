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

PropDriveToWayPoint mode1TurnToPile1;
PropDriveToWayPoint mode1DriveToPile1;
PropDriveToWayPoint mode1TurnToCenter1;
PropDriveToWayPoint mode1DriveToCenter1;

PropDriveToWayPoint progSkillsTurnToRed;
PropDriveToWayPoint progSkillsDriveToWall;
PropDriveToWayPoint progSkillsBackFromWall;
PropDriveToWayPoint progSkillsTurnToGoal;
PropDriveToWayPoint progSkillsBackToWall;
PropDriveToWayPoint progSkillsToShoot;

PropDriveToWayPoint mode2TurnToPile1;
PropDriveToWayPoint mode2DriveToPile1;
PropDriveToWayPoint mode2TurnToCenter1;
PropDriveToWayPoint mode2DriveToShoot1;

PropDriveToWayPoint ivyTurn;
PropDriveToWayPoint ivyDrive;

int isAuto = 1;

long int stepStartTime;

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

	if(alliance == BLUE)
	{
		mode1TurnToPile1 = initPropDriveToWayPoint(robotDrive, 0, -26);
		mode1DriveToPile1 = initPropDriveToWayPoint(robotDrive, 34, 0);
		propDriveToWayPointSetMaxSpeed(&mode1DriveToPile1, 50);
		mode1TurnToCenter1 = initPropDriveToWayPoint(robotDrive, 0, 30);
		mode1DriveToCenter1 = initPropDriveToWayPoint(robotDrive, 20, 0);
	}
	else
	{
		mode1TurnToPile1 = initPropDriveToWayPoint(robotDrive, 0, 26);
		mode1DriveToPile1 = initPropDriveToWayPoint(robotDrive, 34, 0);
		propDriveToWayPointSetMaxSpeed(&mode1DriveToPile1, 50);
		mode1TurnToCenter1 = initPropDriveToWayPoint(robotDrive, 0, -30);
		mode1DriveToCenter1 = initPropDriveToWayPoint(robotDrive, 20, 0);
	}

	progSkillsTurnToRed = initPropDriveToWayPoint(robotDrive, 0, 45);
	progSkillsDriveToWall = initPropDriveToWayPoint(robotDrive, 12*12, 0);
	progSkillsBackFromWall = initPropDriveToWayPoint(robotDrive, -12, 0);
	progSkillsTurnToGoal = initPropDriveToWayPoint(robotDrive, 0, -120);
	progSkillsBackToWall = initPropDriveToWayPoint(robotDrive, -18, 0);
	progSkillsToShoot = initPropDriveToWayPoint(robotDrive, 9, 0);

	if(alliance == BLUE)
	{
		mode2TurnToPile1 = initPropDriveToWayPoint(robotDrive, 0, -26);
		mode2DriveToPile1 = initPropDriveToWayPoint(robotDrive, 34, 0);
		propDriveToWayPointSetMaxSpeed(&mode1DriveToPile1, 50);
		mode2TurnToCenter1 = initPropDriveToWayPoint(robotDrive, 0, 30);
		mode2DriveToShoot1 = initPropDriveToWayPoint(robotDrive, -18, 0);
	}
	else
	{
		mode2TurnToPile1 = initPropDriveToWayPoint(robotDrive, 0, 26);
		mode2DriveToPile1 = initPropDriveToWayPoint(robotDrive, 34, 0);
		propDriveToWayPointSetMaxSpeed(&mode1DriveToPile1, 50);
		mode2TurnToCenter1 = initPropDriveToWayPoint(robotDrive, 0, -30);
		mode2DriveToShoot1 = initPropDriveToWayPoint(robotDrive, -18, 0);
	}

	ivyDrive = initPropDriveToWayPoint(robotDrive, 78, 0);

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
	lcdPrint(uart1, 2, "%d", (int) millis());

	if(autonomousInfo.step != autonomousInfo.lastStep)
	{
		stepStartTime = millis();
	}

	autonomousInfo.elapsedTime = millis() - stepStartTime;

	printf("Step: %d", autonomousInfo.step);

	switch(autonomousSelection)
	{
	case(MODE_1):
				switch(autonomousInfo.step)
				{
				case(1):
					if(autonomousInfo.elapsedTime < 100)
					{
						turnShooterOn(&robotShooter);
					}

					updateShooter(&robotShooter);
					runShooter(&robotShooter);

					autonomousInfo.isFinished = autonomousInfo.elapsedTime > 10000;
					break;

				case(2):
					intake1In(robotIntake);
					intake2In(robotIntake);

					updateShooter(&robotShooter);
					runShooter(&robotShooter);

					autonomousInfo.isFinished = autonomousInfo.elapsedTime > 12000;

					break;

				case(3):
					intake1In(robotIntake);
					intake2Stop(robotIntake);

					if(autonomousInfo.step != autonomousInfo.lastStep)
					{
						shootHalfCourt(&robotShooter);
					}

					updateShooter(&robotShooter);
					runShooter(&robotShooter);

					propDriveToWayPoint(&mode1TurnToPile1);

					autonomousInfo.isFinished = mode1TurnToPile1.isFinished;
					break;

				case(4):
					intake1In(robotIntake);
					intake2Stop(robotIntake);

					updateShooter(&robotShooter);
					runShooter(&robotShooter);

					propDriveToWayPoint(&mode1DriveToPile1);

					autonomousInfo.isFinished = mode1DriveToPile1.isFinished;
					break;

				case(5):
					intake1In(robotIntake);
					intake2Stop(robotIntake);

					updateShooter(&robotShooter);
					runShooter(&robotShooter);

					propDriveToWayPoint(&mode1TurnToCenter1);

					autonomousInfo.isFinished = mode1TurnToCenter1.isFinished;
					break;

				case(6):
					if(autonomousInfo.elapsedTime > 4000)
						intake1In(robotIntake);
					else
						intake1Stop(robotIntake);
					intake2Stop(robotIntake);

					updateShooter(&robotShooter);
					runShooter(&robotShooter);

					propDriveToWayPoint(&mode1DriveToCenter1);

					autonomousInfo.isFinished = mode1DriveToCenter1.isFinished;
					break;

				case(7):
					intake1In(robotIntake);
					intake2In(robotIntake);

					updateShooter(&robotShooter);
					runShooter(&robotShooter);

					autonomousInfo.isFinished = autonomousInfo.elapsedTime > 15000;
					break;

				case(8):
					intake1Stop(robotIntake);
					intake2Stop(robotIntake);

					tankDrive(robotDrive, 0, 0);

					turnShooterOff(&robotShooter);
					updateShooter(&robotShooter);
					runShooter(&robotShooter);

					autonomousInfo.isFinished = !(isAutonomous() && isEnabled());
					break;

				default:
					isAuto = 0;
					break;

				}
	break;

	case(PROG_SKILLS):
			switch(autonomousInfo.step)
			{
			case(1):
				if(autonomousInfo.elapsedTime < 100)
				{
					turnShooterOn(&robotShooter);
				}

				updateShooter(&robotShooter);
				runShooter(&robotShooter);

				autonomousInfo.isFinished = autonomousInfo.elapsedTime > 5000;
				break;

			case(2):
				intake1In(robotIntake);
				intake2In(robotIntake);

				updateShooter(&robotShooter);
				runShooter(&robotShooter);

				autonomousInfo.isFinished = autonomousInfo.elapsedTime > 45000;
				break;

			case(3):
				intake1In(robotIntake);
				intake2In(robotIntake);

				updateShooter(&robotShooter);
				runShooter(&robotShooter);

				propDriveToWayPoint(&progSkillsTurnToRed);

				autonomousInfo.isFinished = progSkillsTurnToRed.isFinished;
				break;

			case(4):
				intake1In(robotIntake);
				intake2In(robotIntake);

				updateShooter(&robotShooter);
				runShooter(&robotShooter);

				propDriveToWayPoint(&progSkillsDriveToWall);

				autonomousInfo.isFinished = progSkillsDriveToWall.isFinished ||
						autonomousInfo.elapsedTime > 6000;
				break;

			case(5):
				intake1In(robotIntake);
				intake2In(robotIntake);

				updateShooter(&robotShooter);
				runShooter(&robotShooter);

				propDriveToWayPoint(&progSkillsBackFromWall);

				autonomousInfo.isFinished = progSkillsBackFromWall.isFinished;
				break;

			case(6):
				intake1In(robotIntake);
				intake2In(robotIntake);

				updateShooter(&robotShooter);
				runShooter(&robotShooter);

				propDriveToWayPoint(&progSkillsTurnToGoal);

				autonomousInfo.isFinished = progSkillsTurnToGoal.isFinished;
				break;

			case(7):
				intake1In(robotIntake);
				intake2In(robotIntake);

				updateShooter(&robotShooter);
				runShooter(&robotShooter);

				propDriveToWayPoint(&progSkillsBackToWall);

				autonomousInfo.isFinished = progSkillsBackToWall.isFinished ||
						autonomousInfo.elapsedTime > 2000;
				break;

			case(8):
				intake1In(robotIntake);
				intake2In(robotIntake);

				updateShooter(&robotShooter);
				runShooter(&robotShooter);

				propDriveToWayPoint(&progSkillsToShoot);

				autonomousInfo.isFinished = progSkillsToShoot.isFinished;
				break;

			default:
				updateShooter(&robotShooter);
				runShooter(&robotShooter);
				break;
			}
			break;

			case(MODE_2):
							switch(autonomousInfo.step)
							{
							case(1):
								if(autonomousInfo.elapsedTime < 100)
								{
									turnShooterOn(&robotShooter);
								}

								updateShooter(&robotShooter);
								runShooter(&robotShooter);

								autonomousInfo.isFinished = autonomousInfo.elapsedTime > 10000;
								break;

							case(2):
								intake1In(robotIntake);
								intake2In(robotIntake);

								updateShooter(&robotShooter);
								runShooter(&robotShooter);

								autonomousInfo.isFinished = autonomousInfo.elapsedTime > 12000;

								break;

							case(3):
								intake1In(robotIntake);
								intake2Stop(robotIntake);

								updateShooter(&robotShooter);
								runShooter(&robotShooter);

								propDriveToWayPoint(&mode2TurnToPile1);

								autonomousInfo.isFinished = mode2TurnToPile1.isFinished;
								break;

							case(4):
								intake1In(robotIntake);
								intake2Stop(robotIntake);

								updateShooter(&robotShooter);
								runShooter(&robotShooter);

								propDriveToWayPoint(&mode2DriveToPile1);

								autonomousInfo.isFinished = mode2DriveToPile1.isFinished;
								break;

							case(5):
								intake1In(robotIntake);
								intake2Stop(robotIntake);

								updateShooter(&robotShooter);
								runShooter(&robotShooter);

								propDriveToWayPoint(&mode2TurnToCenter1);

								autonomousInfo.isFinished = mode2TurnToCenter1.isFinished;
								break;

							case(6):
								if(autonomousInfo.elapsedTime > 4000)
									intake1In(robotIntake);
								else
									intake1Stop(robotIntake);
								intake2Stop(robotIntake);

								updateShooter(&robotShooter);
								runShooter(&robotShooter);

								propDriveToWayPoint(&mode2DriveToShoot1);

								autonomousInfo.isFinished = mode2DriveToShoot1.isFinished;
								break;

							case(7):
								intake1In(robotIntake);
								intake2In(robotIntake);

								updateShooter(&robotShooter);
								runShooter(&robotShooter);

								autonomousInfo.isFinished = autonomousInfo.elapsedTime > 15000;
								break;

							case(8):
								intake1Stop(robotIntake);
								intake2Stop(robotIntake);

								tankDrive(robotDrive, 0, 0);

								turnShooterOff(&robotShooter);
								updateShooter(&robotShooter);
								runShooter(&robotShooter);

								autonomousInfo.isFinished = !(isAutonomous() && isEnabled());
								break;

							default:
								isAuto = 0;
								break;

							}
				break;

				case(IVY):
				switch(autonomousInfo.step)
				{
				case(1):
					if(autonomousInfo.elapsedTime < 100)
					{
						turnShooterOn(&robotShooter);
					}

					updateShooter(&robotShooter);
					runShooter(&robotShooter);

					autonomousInfo.isFinished = autonomousInfo.elapsedTime > 5000;
					break;

				case(2):
					intake1In(robotIntake);
					intake2In(robotIntake);

					updateShooter(&robotShooter);
					runShooter(&robotShooter);

					autonomousInfo.isFinished = autonomousInfo.elapsedTime > 12000;

					break;

				case(3):
					propDriveToWayPoint(&ivyDrive);

					updateShooter(&robotShooter);
					runShooter(&robotShooter);

					autonomousInfo.isFinished = ivyDrive.isFinished;
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

	lcdSetText(uart1, 1, "initialized");

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
