/*
 * cmult.c
 *
 *  Created on: Sep 25, 2020
 *      Author: zhouchang
 */
#include "main.h"

  void cMult(float *arrayA, float *arrayB, float *arrayY, uint32_t blockSize){
		for (uint32_t i = 0; i < blockSize; i++){
			arrayY[i] = arrayA[i] * arrayB[i];
		}
  }

