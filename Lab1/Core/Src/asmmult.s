/*
 * asmmult.s
 *
 *  Created on: Sep 25, 2020
 *      Author: zhouchang
 */

.syntax unified
.global asmMult
.section .text.rodata

/*
 extern void asmMul(float *arrayA, float *arrayB, float *arrayY, uint32_t blockSize);

 R0 = pointer to arrayA
 R1 = pointer to arrayB
 R2 = pointer to arrayY
 R3 = size
*/

asmMult:
	PUSH		{R4, R5, R6}


loop:
	SUBS 		R3, R3, #1
	BLT			done
	ADD			R4, R0, R3, LSL #2
	ADD			R5, R1, R3, LSL #2
	ADD			R6, R2, R3, LSL #2
	VLDR.f32	S0, [R4]
	VLDR.f32	S1, [R5]
	VMUL.f32	S2, S0, S1
	VMRS		APSR_nzvc, FPSCR
	VSTR.f32	S2, [R6]
	B 			loop

done:
	POP			{R4, R5, R6}
	BX			LR
