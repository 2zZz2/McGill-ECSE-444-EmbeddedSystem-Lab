/*
 * lab1math.h
 *
 *  Created on: Sep 25, 2020
 *      Author: zhouchang
 */

#include "main.h"
#ifndef INC_LAB1MATH_H_
#define INC_LAB1MATH_H_
	extern void asmMult(float *arrayA, float *arrayB, float *arrayY, uint32_t blockSize);
	extern void asmSTD(float *array, uint32_t size, float *std);
#endif /* INC_LAB1MATH_H_ */
