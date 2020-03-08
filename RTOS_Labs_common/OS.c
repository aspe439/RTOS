// *************os.c**************
// EE445M/EE380L.6 Labs 1, 2, 3, and 4 
// High-level OS functions
// Students will implement these functions as part of Lab
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 
// Jan 12, 2020, valvano@mail.utexas.edu


#include <stdint.h>
#include <stdio.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "../inc/FIFO.h"
#include "../inc/PLL.h"
#include "../inc/LaunchPad.h"
#include "../inc/Timer4A.h"
#include "../inc/WTimer0A.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../inc/ADCT0ATrigger.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../inc/Timer1A.h"
#include "../inc/Timer2A.h"


// Performance Measurements 
int32_t MaxJitter;             // largest time jitter between interrupts in usec
int32_t MaxJitter2;             // largest time jitter between interrupts in usec

#define JITTERSIZE 64
uint32_t const JitterSize=JITTERSIZE;
uint32_t JitterHistogram[JITTERSIZE]={0,};
uint32_t JitterHistogram2[JITTERSIZE]={0,};

static uint32_t time = 0; // used to track system time

extern void SW1Push(void);
uint32_t FallingEdges = 0;

void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);  // Enable interrupts
void StartOS(void);
void ContextSwitch(void);
void EdgeCounterPortF4_Init(void (*task)(void));
void EdgeCounterPortF0_Init(void(*task)(void));

#define NUMTHREADS  8        // maximum number of threads
#define NUM_SEMA4  4        // maximum number of threads
#define STACKSIZE   128      // number of 32-bit words in stack
extern uint32_t NumCreated;   // number of foreground threads created


//typedef struct tcb* tcbPtr;
tcbType tcbs[NUMTHREADS];
ls semals[NUM_SEMA4][NUMTHREADS];
tcbType *RunPt;
tcbType *NextPt;
AddIndexFifo(os, 16, uint32_t, 1, 0)
//tcbPtr RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];
Sema4Type ST_sema;

void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer. 16 words higher than the bottom of the stack
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit (for PSR)
	//numbers below are all meaningless.
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}

/*------------------------------------------------------------------------------
  Systick Interrupt Handler
  SysTick interrupt happens every 10 ms
  used for preemptive thread switch
 *------------------------------------------------------------------------------*/
void SysTick_Handler(void) {
//	uint32_t status;
//	status = StartCritical();

  OS_Suspend();
//  EndCritical(status);
} // end SysTick_Handler

unsigned long OS_LockScheduler(void){
  // lab 4 might need this for disk formating
  return 0;// replace with solution
}
void OS_UnLockScheduler(unsigned long previous){
  // lab 4 might need this for disk formating
}


void SysTick_Init(unsigned long period){

  long sr = StartCritical();
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0xE0000000; // priority 7
                              // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R = 0x07;
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0xFF00FFFF)|0x00E00000; // PendSV Priority 7
	
  EndCritical(sr);
}

/**
 * @details  Initialize operating system, disable interrupts until OS_Launch.
 * Initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers.
 * Interrupts not yet enabled.
 * @param  none
 * @return none
 * @brief  Initialize OS
 */
void upcount(void){
	long sr = StartCritical();
	time++;
	for (int i = 0; i< NUMTHREADS; i++){ //check sleeping threads, decrease their sleeping time
		if(tcbs[i].SleepTime != 0){
			tcbs[i].SleepTime--;
		}
	}
	 EndCritical(sr);
}

void OS_Init(void){
  // put Lab 2 (and beyond) solution here
  OS_DisableInterrupts();
  PLL_Init(Bus80MHz);         // set processor clock to 50 MHz //50 OR 80?
	UART_Init();
	WideTimer0A_Init(&upcount, 80000, 0);//WideTimer used to keep track of system time
	ST7735_InitR(INITR_REDTAB); // LCD initialization
//  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
//  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
//  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xE0000000; // priority 7
}; 

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, int32_t value){
  // put Lab 2 (and beyond) solution here
	static int i = 0;
	semaPt->Value = value;
	semaPt->blocked = &semals[i][0];
	semaPt->count = i;
	for(int j = 0; j<NUMTHREADS; j++){
		semals[i][j].next = NULL;
		semals[i][j].pointer = NULL;
	}

	i++;
	
}; 

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	long status = StartCritical();
	semaPt->Value--;
	ls *temp = semaPt->blocked;
	if(semaPt->Value < 0){
		int i = 0; 
		if(temp){
			while(temp->next){// temp is the last item in the wait queue
				
				if(temp->pointer == RunPt){
					semaPt ->Value++;
					EndCritical(status);
					OS_Suspend();
					return;
				}
				temp = temp->next;
			}
		}
		for( ;i< NUMTHREADS; i++){ // find a empty spot to put the next item in the linked list queue
			if(semals[semaPt->count][i].pointer == NULL){
				break;
			}
		}
		semals[semaPt->count][i].pointer = RunPt; //set the pointer to current thread
		if(semaPt->Value < -1)
			temp -> next = &semals[semaPt->count][i]; //link the new block in the linked list queue;
		RunPt ->Blocked = 1; //mark the thread as blocked
		EndCritical(status);
		OS_Suspend();
	}
	else 
		EndCritical(status);

	
}; 

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	long status = StartCritical();
		semaPt->Value += 1;
		if(semaPt -> Value <= 0){
			semaPt->blocked->pointer->Blocked = 0;
			semaPt->blocked->pointer = NULL;//mark this node as unused
			
			if(semaPt -> Value <=-1){// if another thread is still blocked.
				semaPt->blocked = semaPt->blocked->next;
			}
			semaPt->blocked->next = NULL;
		}
  EndCritical(status);
}; 

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	DisableInterrupts();


	while(semaPt->Value <= 0){
		EnableInterrupts();
		DisableInterrupts();
	}
	semaPt->Value = 0;
	EnableInterrupts();

}; 

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	long status = StartCritical();
		semaPt->Value = 1;
  EndCritical(status);
}; 



//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), 
   uint32_t stackSize, uint32_t priority)
{
  // put Lab 2 (and beyond) solution here
	int32_t status;
	status = StartCritical();
	
	if(NumCreated > NUMTHREADS-1){
		return 0;
	}
	
	uint16_t i = 0;
	if(NumCreated == 0){								//Didn't consider the situation where all threads are killed.
		i = 0;
		tcbs[i].next = &tcbs[0];
		tcbs[i].Id = 1;	
	}
	else{		
		while(tcbs[i].Id != 0){	//search for an unassigned thread slot.
			i++;
		}	
		tcbs[i-1].next = &tcbs[i];
		uint16_t k = i;
		while((tcbs[k].Id == 0)&&(k != 0)){	//search for next assigned thread slot.
			k++;
			if(k >= NUMTHREADS){
				k = 0;
			}
		}
		tcbs[i].next = &tcbs[k];
		tcbs[i].Id = i+1;
	}
	
	SetInitialStack(i); Stacks[i][STACKSIZE-2] = (int32_t)(task); // PC
  EndCritical(status);
  return 1; // replace this line with solution
};

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
uint32_t OS_Id(void){
  // put Lab 2 (and beyond) solution here
	
  
  return RunPt->Id; // replace this line with solution
};


//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In lab 1, this command will be called 1 time
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddPeriodicThread(void(*task)(void), 
   uint32_t period, uint32_t priority)
{
	static char count = 0;
	if(count == 0){
		 Timer2A_Init(task, period, priority);  // initialize timer2A (1000 Hz)
		count = 1;
	}
	if(count == 1){
		 Timer1A_Init(task, period, priority);
	}
  // put Lab 2 (and beyond) solution here
 
  return 0;
};




/*----------------------------------------------------------------------------
  PF0 and PF4 Interrupt Handler
 *----------------------------------------------------------------------------*/
void  (*GPIOF4HandlerFunction)(void);
void  (*GPIOF0HandlerFunction)(void);
void GPIOPortF_Handler(void){
	long sr = StartCritical();
	if((GPIO_PORTF_RIS_R&0x10)==0x10){
	(*GPIOF4HandlerFunction)();
	GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4
  FallingEdges = FallingEdges + 1;
	}
	if((GPIO_PORTF_RIS_R&0x01)==0x01){
	(*GPIOF0HandlerFunction)();
	GPIO_PORTF_ICR_R = 0x01;      // acknowledge flag4
  FallingEdges = FallingEdges + 1;
	}
	EndCritical(sr);
}
//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW1Task(void(*task)(void), uint32_t priority){
  // put Lab 2 (and beyond) solution here
	EdgeCounterPortF4_Init(task);
  return 0; // replace this line with solution
};

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), uint32_t priority){
  // put Lab 2 (and beyond) solution here
  EdgeCounterPortF0_Init(task);
  return 0; // replace this line with solution
};

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
  // put Lab 2 (and beyond) solution here
   RunPt->SleepTime = sleepTime;
	 OS_Suspend();

};  

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
int foo = 0;
void OS_Kill(void){
  // put Lab 2 (and beyond) solution here
	long status = StartCritical();
	uint32_t RunningThId = OS_Id();
//	int32_t i = RunningThId-1;
	tcbType* temp1 = &tcbs[RunningThId-1];
	tcbType* temp2 = tcbs[RunningThId-1].next;//Store the next thread on linked list.
	//temp1->next = 0;
	uint32_t k = 0;
	for(; k < NUMTHREADS;k++){
		foo = 0;
		if(tcbs[k].next == temp1){
			foo = 1;
			break;
		}	
	}
	tcbs[k].next = temp2;		//Update the preceding item on linked list.
	NumCreated--;
	temp1->Id = 0;
	temp1->sp = &Stacks[RunningThId-1][STACKSIZE-1];//ready to release the stack.
	EndCritical(status);   // end of atomic section .
	//Todo: where should I enable interrupt again?
	OS_Suspend();
	for(;;){};        // can not return
    
}; 

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void){
  // put Lab 2 (and beyond) solution here
	NextPt = RunPt->next;
	while((NextPt->SleepTime!=0)||(NextPt->Blocked==1)){
		NextPt = NextPt -> next; //loop through the list until an active task is found
	}
	//UART_OutString("Switch!\r\n");
  ContextSwitch();
	//UART_OutString("Switch DOne!\r\n");

};
  
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
Sema4Type room_left;
Sema4Type data_s;
void OS_Fifo_Init(uint32_t size){
  // put Lab 2 (and beyond) solution here
	osFifo_Init();
  OS_InitSemaphore(&room_left, 8);
	OS_InitSemaphore(&data_s, 0);
  
};

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(uint32_t data){
  // put Lab 2 (and beyond) solution here
		long status;
		if(room_left.Value <= 0 ){
			return 0;
		}
		else{
			status = StartCritical();
			room_left.Value --;
			
		  osFifo_Put(data);
			data_s.Value++; 
		}
		
		
		EndCritical(status);
    return 1; // replace this line with solution
};  

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
uint32_t OS_Fifo_Get(void){
  // put Lab 2 (and beyond) solution here
	OS_Wait(&data_s);
//	uint32_t FIFOout;
	uint32_t temp = 0;
	osFifo_Get(&temp);
	OS_Signal(&room_left);	
  return temp; // replace this line with solution
};

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
int32_t OS_Fifo_Size(void){
  // put Lab 2 (and beyond) solution here
   
  return 0; // replace this line with solution
};


// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
Sema4Type *insema;
Sema4Type *outsema;
uint32_t mailbox;
void OS_MailBox_Init(void){
  // put Lab 2 (and beyond) solution here
  OS_InitSemaphore(insema, 0);
	OS_InitSemaphore(outsema, 0);

  // put solution here
};

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(uint32_t data){
  // put Lab 2 (and beyond) solution here
  // put solution here
   OS_bWait(outsema);
	 mailbox = data;
	 OS_bSignal(insema);

};

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
uint32_t OS_MailBox_Recv(void){
  // put Lab 2 (and beyond) solution here
	OS_bWait(insema);
	OS_bSignal(outsema);
  return mailbox; // replace this line with solution
};

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
uint32_t OS_Time(void){
  // put Lab 2 (and beyond) solution here
	
  return time * 80000 + (80000 - WTIMER0_TAR_R); // replace this line with solution
};

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
uint32_t OS_TimeDifference(uint32_t start, uint32_t stop){
  // put Lab 2 (and beyond) solution here

  return (stop -start); // replace this line with solution
};


// ******** OS_ClearMsTime ************
// sets the system time to zero (solve for Lab 1), and start a periodic interrupt
// Inputs:  none
// Outputs: none
// You are free to change how this works

void OS_ClearMsTime(void){
	
  // put Lab 1 solution here
	time = 0; //initialized when OS gets initilized
};

// ******** OS_MsTime ************
// reads the current time in msec (solve for Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// For Labs 2 and beyond, it is ok to make the resolution to match the first call to OS_AddPeriodicThread
uint32_t OS_MsTime(void){
  // put Lab 1 solution here
  return time; 
};


//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(uint32_t theTimeSlice){
  // put Lab 2 (and beyond) solution here
	SysTick_Init(theTimeSlice);
	TIMER2_CTL_R = 0x00000001;    // 10) enable TIMER1A
	TIMER1_CTL_R = 0x00000001;    // 10) enable TIMER1A
	RunPt = &tcbs[0];
  StartOS();                   // start on the first task     
};

//******** I/O Redirection *************** 
// redirect terminal I/O to UART

int fputc (int ch, FILE *f) { 
  UART_OutChar(ch);
  return ch; 
}

int fgetc (FILE *f){
  char ch = UART_InChar();  // receive from keyboard
  UART_OutChar(ch);         // echo
  return ch;
}

int OS_RedirectToFile(char *name){
  
  return 1;
}
int OS_RedirectToUART(void){
  
  return 1;
}

int OS_RedirectToST7735(void){
  
  return 1;
}

int OS_EndRedirectToFile(void){
  
  return 1;
}


void OS_jittermeasurement(uint32_t PERIOD, uint32_t* JitterHistogram, unsigned long* LastTime){
	unsigned long jitter; 
	uint32_t thisTime = OS_Time();
	uint32_t diff = OS_TimeDifference(*LastTime,thisTime);
      if(diff>PERIOD){
        jitter = (diff-PERIOD+4)/8;  // in 0.1 usec
      }else{
        jitter = (PERIOD-diff+4)/8;  // in 0.1 usec
      }
      if(jitter > MaxJitter){
        MaxJitter = jitter; // in usec
      }       // jitter should be 0
      if(jitter >= JitterSize){
        jitter = JitterSize-1;
      }
      JitterHistogram[jitter]++; 
	*LastTime = thisTime; 
}

void EdgeCounterPortF4_Init(void(*task)(void)){                          
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
	GPIOF4HandlerFunction = task;
  FallingEdges = 0;             // (b) initialize counter
  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4   
  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}

void EdgeCounterPortF0_Init(void(*task)(void)){                          
	GPIOF0HandlerFunction = task;
  FallingEdges = 0;             // (b) initialize counter
  GPIO_PORTF_DIR_R &= ~0x01;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x01;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x01;     //     enable digital I/O on PF4   
  GPIO_PORTF_PCTL_R &= ~0x0000000F; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x01;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x01;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x01;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x01;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x01;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x01;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}
