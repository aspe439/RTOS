// *************Interpreter.c**************
// Students implement this as part of EE445M/EE380L.12 Lab 1,2,3,4 
// High-level OS user interface
// 
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 1/18/20, valvano@mail.utexas.edu
#include <stdint.h>
#include <string.h> 
#include <stdio.h>
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../inc/ADCT0ATrigger.h"
#include "../inc/ADCSWTrigger.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"

static char things[50] = {0};
// Print jitter histogram
void Jitter(int32_t MaxJitter, uint32_t const JitterSize, uint32_t JitterHistogram[]){
  // write this for Lab 3 (the latest)
	UART_OutString("\r\nMaxJitter\n\r");
	UART_OutUDec(MaxJitter);
	UART_OutString("\r\nJitterHistogram:\n\r");
	for(uint32_t i=0;i<JitterSize;i++){
		UART_OutString("\r\nJitter in 0.1 usec: ");
		UART_OutUDec(i);
		UART_OutString("\r\nFrequency: ");
		UART_OutUDec(JitterHistogram[i]);
	}
	UART_OutString("\r\nJitterHistogram:\n\r");
}
extern uint32_t MaxJitter;   // number of foreground threads created
extern uint32_t DataLost;      // current number of PID calculations finished
extern uint32_t FilterWork;   // number of digital filter calculations finished
extern uint32_t NumSamples;   // incremented every ADC sample, in Producer
// *********** Command line interpreter (shell) ************
void Interpreter(void){ 
  // write this  
	while(1){
//	UART_InString(&things[0], 50); //buffer to store input string
//	if(strcmp(things, "Performance") == 0){
//		UART_OutString("\r\nMaxJitter\n\r");
//		UART_OutUDec(MaxJitter);
//		UART_OutString("\r\nDataLost\n\r");
//		UART_OutUDec(DataLost);
//		UART_OutString("\r\nFilterWork\n\r");
//		UART_OutUDec(FilterWork);
//		UART_OutString("\r\nNumSamples\n\r");
//		UART_OutUDec(NumSamples);
//	}
//	things[0] = '\0'; //clears the command buffer
}
//	if(strcmp(things, "-help") == 0){ //help promp
//		UART_OutString("List of possible command: \r\n\
//-clr to clear the terminal\r\n\
//-lcd to select the output to lcd\r\n\
//	-clr to clear the lcd \r\n\
//	-up to select upper screen\r\n\
//	-low to select lower screen\r\n\
//		-l to specify line number, default is 0\r\n\
//		-adc to print ADC value\r\n\
//		-time to print system ellapsed time \r\n\
//-uart to print the output to uart terminal \r\n\
//	-adc to print ADC value\r\n\
//	-timer to prnit system ellapsed time \r\n\
//	any other command to print those charaters on the selected display\r\n");
//	}
//	else if(strcmp(things, "-clr")==0){ //clear screen command
//		UART_OutString("\033[2J\033[;H\33[0m");	 //clears screen, move cursor to topleft using VT100 command
//		UART_OutString("Welcome to the RTOS lab 1 parser! /r/n Please input you command, type -help for more info regarding commands. \n\r");

//	}
//  else if(strcmp(things, "-lcd")==0){ //if lcd is selected
//		UART_OutString("\n\rdisplaying on LCD\n\ruse -up or -low to select upper or lower screen or use -clr to clear the LCD\n\r");	 //clears screen, move cursor to topleft using VT100 command
//		things[0] = '\0'; //clears the command buffer
//		UART_InString(&things[0], 50); //read the next command
//		if (strcmp(things, "-clr")==0){
//			ST7735_FillScreen(0); //clear the LCD screen
//			UART_OutString("\r\nLCD cleared \n\r");
//		}
//		else if(strcmp(things, "-up")==0){ //if upper LCD display is selected
//			int line = 0;
//			UART_OutString("\r\nupper screen selected\r\n select display content\n\ruse -l to specify line number(default 0) or -adc and -time to specfy display content");

//			things[0] = '\0'; //clears the command buffer
//			UART_InString(&things[0], 50); //read the next command
//			if(strcmp(things, "-l")==0){ // if user specifying line number
//				UART_OutString("\r\ninput line number, between 0 - 7 \r\n");
//				line = UART_InUHex(); //get line number
//				if(line<0||line>7){// if line number invalid
//					UART_OutString("\r\ninvalid line number, revert to default");
//					line = 0;
//				}
//				UART_OutString("\r\nuse -adc or -time to select display content, or type in content to be displayed\n\r");
//				things[0] = '\0'; //clears the command buffer
//				UART_InString(&things[0], 50); //read the next command
//			}
//			if(strcmp(things, "-adc")==0){ //if ADC is to be displayed on lcd
//				ST7735_SetCursor(0,line); //set cursor to the line to be used
//				ST7735_OutString("                    "); //clear the line 
//				ST7735_Message((uint32_t) 0, (uint32_t) line, "ADC Reading is", (int32_t) adc);
//				UART_OutString("\r\nDone!\n\r");

//			}
//			else if(strcmp(things, "-time")==0){
//				ST7735_SetCursor(0,line); //set cursor to the line to be used
//				ST7735_OutString("                    "); //clear the line 
//				ST7735_Message((uint32_t) 0, (uint32_t) line, "System time in ms is", (int32_t) time);
//				UART_OutString("\r\nDone!\n\r");
//			}
//			else{
//				ST7735_SetCursor(0,line); //set cursor to the line to be used
//				ST7735_OutString("                    "); //clear the line 
//				ST7735_SetCursor(0,line);
//				ST7735_OutString(things);
//				UART_OutString("\r\nDone!\n\r");
//			}
//		}
//	
//		else if(strcmp(things, "-low")==0){
//			int line = 8;
//			UART_OutString("\r\nlower screen selected\r\n select display content\n\ruse -l to specify line number(default 0) or -adc and -time to specfy display content");
//			things[0] = '\0'; //clears the command buffer
//			UART_InString(&things[0], 50); //read the next command
//			if(strcmp(things, "-l")==0){ // if user specifying line number
//				UART_OutString("\r\ninput line number, between 0 - 7 \r\n");
//				line = UART_InUHex(); //get line number
//				if(line<0||line>7){// if line number invalid
//					UART_OutString("\r\ninvalid line number, revert to default");
//					line = 0;
//				}
//				UART_OutString("\r\nuse -adc or -time to select display content, or type in content to be displayed\n\r");
//				things[0] = '\0'; //clears the command buffer
//				UART_InString(&things[0], 50); //read the next command
//			}
//			if(strcmp(things, "-adc")==0){ //if ADC is to be displayed on lcd
//				ST7735_SetCursor(0,line); //set cursor to the line to be used
//				ST7735_OutString("                    "); //clear the line 
//				ST7735_Message((uint32_t) 1, (uint32_t) line-8, "ADC Reading is", (int32_t) adc);
//				UART_OutString("\r\nDone!\n\r");

//			}
//			else if(strcmp(things, "-time")==0){
//				ST7735_SetCursor(0,line); //set cursor to the line to be used
//				ST7735_OutString("                    "); //clear the line 
//				ST7735_Message((uint32_t) 1, (uint32_t) line-8, "System time in ms is", (int32_t) time);
//				UART_OutString("\r\nDone!\n\r");
//			}
//			else{
//				ST7735_SetCursor(0,line); //set cursor to the line to be used
//				ST7735_OutString("                    "); //clear the line 
//				ST7735_SetCursor(0,line);
//				ST7735_OutString(things);
//				UART_OutString("\r\nDone!\n\r");
//			}
//		}
//	}
//	else if(strcmp(things, "-uart")==0){ //if lcd is selected
//				UART_OutString("\r\nuse -adc or -time to select display content, or type in content to be displayed\n\r");
//				things[0] = '\0'; //clears the command buffer
//				UART_InString(&things[0], 50); //read the next command
//			if(strcmp(things, "-adc")==0){ //if ADC is to be displayed on lcd
//				UART_OutString("\r\n ADC reading is ");
//				UART_OutUDec(adc);
//				UART_OutString(" which is ");
//				UART_OutUDec(adc*1000/4096);
//				UART_OutString(" mV\r\n");


//	
//			}
//			else if(strcmp(things, "-time")==0){
//				UART_OutString("\r\nSystem time in ms is ");
//				UART_OutUDec(time);
//				UART_OutString(" ms\n\r");
//			}
//			else{
//				UART_OutString("\r\n Echoing  ");
//				UART_OutString(things);
//				
//			}
//	}
//	else{ 
//		UART_OutString("\r\ninvalid input: ");
//		UART_OutString(things);
//		UART_OutString("\n\r");
//	}
//	things[0] = '\0'; //clears the command buffer
}


