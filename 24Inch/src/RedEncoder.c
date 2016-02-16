/*
 * RedEncoder.c
 *
 *  Created on: Feb 10, 2016
 *      Author: Erik
 */

#include "main.h"

RedEncoder initRedEncoder(Encoder encoder, long refreshTime)
{
	RedEncoder newEncoder = {encoder, micros(), encoderGet(encoder), 0};
	return newEncoder;
}

int getRedEncoder(RedEncoder *encoder)
{
	return encoderGet((*encoder).encoder);
}

double getRedEncoderVelocity(RedEncoder *encoder)
{
	if(micros() - (*encoder).lastReadTime > 100000)
	{
		int currentEncoder = encoderGet((*encoder).encoder);

		double velocity = (double) ((double) (currentEncoder - (*encoder).lastEncoder) /
				(double) (micros() - (*encoder).lastReadTime));

		(*encoder).lastEncoder = encoderGet((*encoder).encoder);

		(*encoder).lastReadTime = micros();

		(*encoder).velocity = velocity * 100000;
	}

	return (*encoder).velocity;
}
