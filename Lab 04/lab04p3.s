// file: lab04p3.s
// description lab04 p3
// author: ado maksic and davis garman

.global main
main:

.section  .text
  

    push {r4,r5,r6,r7}

      ldr   r4,=DS // Load 'DS' into r4
      ldrb  r5,[r4,#(A-DS)] // Load A into r5
      ldrh  r6,[r4,#(B-DS)] // Load B into r6
      add   r7,r5,r6 // Add r5 and r6
      str   r7,[r4,#(C-DS)] // Store value of C into r7

    all_done: nop

    pop {r4,r5,r6,r7}
    
      bx lr

      
  .section .data

  .org 234
  .align 2,0xa5
  
DS:  .word 0xbbbbbbbb

A:   .byte 123

     .align 1,0xa5	 
B:   .short 47587

     .align 2,0xa5	 
C:   .word ~0

     .align 3,0xa5
     .word 0xeeeeeeee

  .end