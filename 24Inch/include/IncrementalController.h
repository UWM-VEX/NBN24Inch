/*
 * IncrementalController.h
 *
 *  Created on: Feb 10, 2016
 *      Author: Erik
 */

#ifndef INCLUDE_INCREMENTALCONTROLLER_H_
#define INCLUDE_INCREMENTALCONTROLLER_H_

#include "main.h"

struct IncrementalController{

	RedEncoder *encoder;
	long lastReadTime;
	double output;
	double error;

}typedef IncrementalController;

IncrementalController initIncrementalController(RedEncoder *encoder);
int runIncrementalController(IncrementalController *controller, double setPoint);

#endif /* INCLUDE_INCREMENTALCONTROLLER_H_ */
