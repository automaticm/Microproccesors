// file: lab04p2b.s
// description lab04 p2b
// author: ado maksic and davis garman


.global _start
_start:
	
.section .text
	push {r4, r5, r6, r7}
	
	
	ldr r4, =DS // Loads the value 'DS" in r4
	ldrb r5, [r4, #(B-DS)] // Loads the data referenced from label 'B' into r5
	ldr r6, [r4, #(W-DS)] // Loads the data referenced from label 'W' into r6
	
	all_done: nop
	
	pop {r4, r5, r6, r7}
		bx lr
		

.section .data
	.org 300
	.align 2, 0xaa
	
	.word 0xbbbbbbbb

B: .byte 85
	.align 2, 0xaa
	
DS: .word ~0
	.align 2, 0xaa
	
W: 	.word 287454020
	.word 0xeeeeeeee
	
.end