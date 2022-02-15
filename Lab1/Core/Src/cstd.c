/*
 * cstd.c
 *
 *  Created on: Sep 25, 2020
 *      Author: zhouchang
 */
#include "main.h"

void cSTD(float *array, uint32_t size, float *std){
	float mean = 0;
	float variance = 0;
	for (uint32_t i = 0; i < size; i++){
		mean = mean + array[i]/size;
	}
	for (uint32_t i = 0; i < size; i++){
		variance = variance + (array[i]-mean)*(array[i]-mean)/(size-1);
	}
	(*std) = sqrt(variance);
}
