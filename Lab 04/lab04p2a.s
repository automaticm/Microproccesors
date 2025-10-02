// file: lab04p2a.s
// description lab04 p2a
// author: ado maksic and davis garman


.global _start
_start:
	
.section .text
	push {r4, r5, r6, r7}
	
	
	ldr r4, =W // Loads the value of W in r4
	ldr r5, [r4, #(W-DSECT)] // Loads the data referenced from label W into r5
	
	all_done: nop
	
	pop {r4, r5, r6, r7}
		bx lr
		
	.section .data
	
DSECT:	.word 0xbbbbbbbb
		
W:		.word 4277009102
		
		.word 0xeeeeeeee