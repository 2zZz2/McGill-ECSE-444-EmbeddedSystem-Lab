/*
 * asmstd.s
 *
 *  Created on: Sep 25, 2020
 *      Author: zhouchang
 */

.syntax unified
.global asmSTD
.section .text.rodata

/*
 * void asmSTD(float *array, uint32_t size, float *std);
 * R0 = pointer to array
 * R1 = size
 * R2 = pointer to result std
 *
 */

asmSTD:
	PUSH			{R4, R5}
	MOV 			R4, R1		//counter for the loop
	MOV 			R5, #0		//comtemperorily store data
	VMOV.f32		S1, R5		//initialize the mean to 0
	VCVT.f32.s32	S1, S1
	VMOV.f32		S2, R4		//initialize the size
	VCVT.f32.s32	S2, S2
	VMOV.f32		S3, R6		//set the variance to 0
	VCVT.f32.s32	S3, S3

findMean:
	SUBS			R4, R4, #1
	BLT				resetCounter
	ADD				R5, R0, R4, LSL #2
	VLDR.f32		S0, [R5]	//get raw data from the float array
	VDIV.f32		S0,	S0, S2	//scale down the data to data/size
	VADD.f32		S1, S0, S1	//S1 = mean = sigma(data(i)/size), i grows from 1 to the size
	VMRS			APSR_nzvc, FPSCR
	B				findMean

resetCounter:
	MOV 			R4, R1
	SUB				R4, R4, #1
	MOV				R5, #0
	VMOV.f32		S2, R4	//S2 = size-1
	VCVT.f32.s32	S2, S2
	VMOV.f32		S0, R5	//clear S0
	VCVT.f32.s32	S0, S0
	ADD				R4, R4, #1

findVariance:
	SUBS			R4, R4, #1
	BLT				done
	ADD				R5, R0, R4, LSL #2
	VLDR.f32		S0, [R5]	//get raw data from the float array
	VSUB.f32		S0, S0, S1	//S0 = data(i)-mean
	VMUL.f32		S0, S0, S0	//S0 = (data(i)-mean)^2
	VDIV.f32		S0,	S0, S2	//S0 = ((data(i)-mean)^2)/(size-1)
	VADD.f32		S3, S3, S0 	//S3 = sigma(((data(i)-mean)^2)/(size-1)), i from 1 to size
	VMRS			APSR_nzvc, FPSCR
	B				findVariance

done:
	VSQRT.f32		S2, S3		//take square root S2 = sqrt(S3)
	VSTR.f32		S2, [R2]
	POP				{R4, R5}
	BX LR
