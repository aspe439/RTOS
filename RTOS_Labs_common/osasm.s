;/*****************************************************************************/
;/* OSasm.s: low-level OS commands, written in assembly                       */
;/* derived from uCOS-II                                                      */
;/*****************************************************************************/
;Jonathan Valvano, OS Lab2/3/4, 1/12/20
;Students will implement these functions as part of EE445M/EE380L.12 Lab

        AREA |.text|, CODE, READONLY, ALIGN=2
        THUMB
        REQUIRE8
        PRESERVE8

        EXTERN  RunPt            ; currently running thread
        EXTERN  NextPt           ; Next active thread
		
        EXPORT  StartOS
        EXPORT  ContextSwitch
        EXPORT  PendSV_Handler
		EXPORT  OS_DisableInterrupts
        EXPORT  OS_EnableInterrupts


NVIC_INT_CTRL   EQU     0xE000ED04                              ; Interrupt control state register.
NVIC_SYSPRI14   EQU     0xE000ED22                              ; PendSV priority register (position 14).
NVIC_SYSPRI15   EQU     0xE000ED23                              ; Systick priority register (position 15).
NVIC_LEVEL14    EQU           0xEF                              ; Systick priority value (second lowest).
NVIC_LEVEL15    EQU           0xFF                              ; PendSV priority value (lowest).
NVIC_PENDSVSET  EQU     0x10000000                              ; Value to trigger PendSV exception.


OS_DisableInterrupts
        CPSID   I
        BX      LR


OS_EnableInterrupts
        CPSIE   I
        BX      LR

StartOS
; put your code here
;	LDR		R0, =NVIC_LEVEL15
;	LDR		R2, =NVIC_SYSPRI14
;	STR		R0, [R2]
;	LDR		R0, =NVIC_LEVEL14
;	LDR		R2, =NVIC_SYSPRI15
;	STR		R0, [R2]
    LDR     R0, =RunPt         ; currently running thread. the address of Runpt
    LDR     R2, [R0]           ; R2 = value of RunPt
    LDR     SP, [R2]           ; new thread SP; SP = RunPt->stackPointer; What is this?
    POP     {R4-R11}           ; restore regs r4-11
    POP     {R0-R3}            ; restore regs r0-3
    POP     {R12}
    POP     {LR}               ; discard LR from initial stack
    POP     {LR}               ; start location PC
    POP     {R1}               ; discard PSR. Because now we don't need PSR.
    CPSIE   I                  ; Enable interrupts at processor level
    
    BX      LR                 ; start first thread

OSStartHang
    B       OSStartHang        ; Should never get here


;********************************************************************************************************
;                               PERFORM A CONTEXT SWITCH (From task level)
;                                           void ContextSwitch(void)
;
; Note(s) : 1) ContextSwitch() is called when OS wants to perform a task context switch.  This function
;              triggers the PendSV exception which is where the real work is done.
;********************************************************************************************************

ContextSwitch
; edit this code
    LDR		R0, =NVIC_INT_CTRL
	LDR     R1, =NVIC_PENDSVSET
	STR		R1, [R0]
    BX      LR
    

;********************************************************************************************************
;                                         HANDLE PendSV EXCEPTION
;                                     void OS_CPU_PendSVHandler(void)
;
; Note(s) : 1) PendSV is used to cause a context switch.  This is a recommended method for performing
;              context switches with Cortex-M.  This is because the Cortex-M3 auto-saves half of the
;              processor context on any exception, and restores same on return from exception.  So only
;              saving of R4-R11 is required and fixing up the stack pointers.  Using the PendSV exception
;              this way means that context saving and restoring is identical whether it is initiated from
;              a thread or occurs due to an interrupt or exception.
;
;           2) Pseudo-code is:
;              a) Get the process SP, if 0 then skip (goto d) the saving part (first context switch);
;              b) Save remaining regs r4-r11 on process stack;
;              c) Save the process SP in its TCB, OSTCBCur->OSTCBStkPtr = SP;
;              d) Call OSTaskSwHook();
;              e) Get current high priority, OSPrioCur = OSPrioHighRdy;
;              f) Get current ready thread TCB, OSTCBCur = OSTCBHighRdy;
;              g) Get new process SP from TCB, SP = OSTCBHighRdy->OSTCBStkPtr;
;              h) Restore R4-R11 from new process stack;
;              i) Perform exception return which will restore remaining context.
;
;           3) On entry into PendSV handler:
;              a) The following have been saved on the process stack (by processor):
;                 xPSR, PC, LR, R12, R0-R3
;              b) Processor mode is switched to Handler mode (from Thread mode)
;              c) Stack is Main stack (switched from Process stack)
;              d) OSTCBCur      points to the OS_TCB of the task to suspend
;                 OSTCBHighRdy  points to the OS_TCB of the task to resume
;
;           4) Since PendSV is set to lowest priority in the system (by OSStartHighRdy() above), we
;              know that it will only be run when no other exception or interrupt is active, and
;              therefore safe to assume that context being switched out was using the process stack (PSP).
;********************************************************************************************************

PendSV_Handler
; put your code here
	CPSID	I				 ; disable interrupt
	PUSH	{R4-R11}		 ; push remaining registers onto the stack
	LDR		R0, =RunPt		 ; R0=pointer to RunPt, old
	LDR     R1, [R0]		 ; R1 = RunPt
	STR		SP, [R1]		 ; Save SP into TCB
	LDR		R0, =NextPt      ; NextPt is the next task that is ready
	LDR		R1, [R0]         ; Load NextPt into R1
	LDR		R0, =RunPt		 ; R0=pointer to RunPt, old
	STR		R1, [R0]
	LDR		SP, [R1]
	POP		{R4-R11}
	CPSIE	I 				   ; enable interrupt
		BX      LR                 ; Exception return will restore remaining context
							   ; Restore R0-R3,R12,LR,PC,PSR automatically beacuse it is an interrupt.
    


    ALIGN
    END
