/* use code segment, and we are generating ARM thumb code: */
  RSEG    CODE:CODE(2)
  thumb

  /* external interface declaration; */
  PUBLIC __HwFault



        PUBWEAK HandlerC
        SECTION .text:CODE:REORDER:NOROOT(1)
HandlerC
        B .

/*-----------------------------------------------------------*/
__HwFault
  movs r0,#4       /* load bit mask into R0 */
  mov r1, lr       /* load link register into R1 */
  tst r0, r1       /* compare with bitmask */
  beq _MSP         /* if bitmask is set: stack pointer is in PSP. Otherwise in MSP */
  mrs r0, psp      /* otherwise: stack pointer is in PSP */
  b _GetPC         /* go to part which loads the PC */
  _MSP:            /* stack pointer is in MSP register */
  mrs r0, msp      /* load stack pointer into R0 */
  _GetPC:          /* find out where the hard fault happened */
  ldr r1,[r0,#20]  /* load program counter into R1. R1 contains address of the next instruction where the hard fault happened */
  b HandlerC       /* decode more information. R0 contains pointer to stack frame */
/*-----------------------------------------------------------*/
  END