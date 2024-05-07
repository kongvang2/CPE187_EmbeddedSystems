// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)


// Kong Meng Vang

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"
//#include "PLL.h"
//#include "SysTick.h"

#define LIGHT                   (*((volatile unsigned long *)0x400050FC))
#define GPIO_PORTB_OUT          (*((volatile unsigned long *)0x400050FC)) // bits 5-0
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define GPIO_PORTE_IN           (*((volatile unsigned long *)0x4002401C)) // bits 2-0
#define SENSOR                  (*((volatile unsigned long *)0x4002401C))

#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // port B Clock Gating Control

#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define PF31 										(*((volatile unsigned long *)0x40025028))

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

void PortFBE_Init(void);
//Program 10.1. Activate the LM4F/TM4C with a 16 MHz crystal to run at 80 MHz
//void PLL_Init(void);
void SysTick_Init(void);	
void SysTick_Wait(unsigned long);
void SysTick_Wait10ms(unsigned long);

// ***** 3. Subroutines Section *****




//Linked Data Structure
struct State {
	unsigned long Out; //6-LED output for the state
	unsigned long Walk; //Walk/DontWalk LED Output
	unsigned long Time; //wait time
	unsigned long Next[8]; //next state that is dependent upon inputs
};
typedef const struct State STyp;

#define	GoWest 			0
#define	WaitWest 		1
#define	StopWest 		2
#define	GoSouth			3
#define	WaitSouth 	4
#define	StopSouth		5
#define	WalkStart		6
#define	DontWalkOn1 7
#define WalkOff1 		8
#define	DontWalkOn2	9
#define WalkOff2		10
#define DontWalkOn3 11
#define WalkOff3		12
#define	WalkEnd			13


// Initialize SysTick with busy wait running at bus clock.
#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))
	
STyp FSM[14]={
	{0x0C,0x02,50,{GoWest,GoWest,WaitWest,WaitWest,WaitWest,WaitWest,WaitWest,WaitWest}}, //0
	{0x14,0x02,30,{StopWest,StopWest,StopWest,StopWest,StopWest,StopWest,StopWest,StopWest}}, //1
	{0x24,0x02,30,{GoWest,GoWest,GoSouth,GoSouth,WalkStart,WalkStart,WalkStart,GoSouth}}, //2
  {0x21,0x02,50,{GoSouth,WaitSouth,GoSouth,WaitSouth,WaitSouth,WaitSouth,WaitSouth,WaitSouth}}, //3
	{0x22,0x02,30,{StopSouth,StopSouth,StopSouth,StopSouth,StopSouth,StopSouth,StopSouth,StopSouth}}, //4
	{0x24,0x02,30,{GoSouth,GoWest,GoSouth,GoWest,WalkStart,WalkStart,WalkStart,WalkStart}}, //5
	{0x24,0x08,50,{DontWalkOn1,DontWalkOn1,DontWalkOn1,DontWalkOn1,DontWalkOn1,DontWalkOn1,DontWalkOn1,DontWalkOn1}}, //6
	{0x24,0x02,10,{WalkOff1,WalkOff1,WalkOff1,WalkOff1,WalkOff1,WalkOff1,WalkOff1,WalkOff1}}, //7
	{0x24,0x00,10,{DontWalkOn2,DontWalkOn2,DontWalkOn2,DontWalkOn2,DontWalkOn2,DontWalkOn2,DontWalkOn2,DontWalkOn2}}, //8
	{0x24,0x02,10,{WalkOff2,WalkOff2,WalkOff2,WalkOff2,WalkOff2,WalkOff2,WalkOff2,WalkOff2}}, //9
	{0x24,0x00,10,{DontWalkOn3,DontWalkOn3,DontWalkOn3,DontWalkOn3,DontWalkOn3,DontWalkOn3,DontWalkOn3,DontWalkOn3}}, //10
  {0x24,0x02,10,{WalkOff3,WalkOff3,WalkOff3,WalkOff3,WalkOff3,WalkOff3,WalkOff3,WalkOff3}}, //11
	{0x24,0x00,10,{WalkEnd,WalkEnd,WalkEnd,WalkEnd,WalkEnd,WalkEnd,WalkEnd,WalkEnd}}, //12
	{0x24,0x02,10,{GoWest,GoWest,GoSouth,GoWest,WalkStart,GoWest,GoSouth,GoWest}} //13	
};


unsigned long CurrState; //index to the current state 
unsigned long Input;
	
int main(void){ 
	//volatile unsigned long delay;
	
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	
	PortFBE_Init();
	//PLL_Init();       // 80 MHz, Program 10.1
  SysTick_Init();   // Program 10.2
 
  
  EnableInterrupts();
	
	//set default state to GoWest
	CurrState = GoWest;
	
  while(1){
		LIGHT = FSM[CurrState].Out;							//set the 6-LED lights
		PF31 = FSM[CurrState].Walk;							//set the Walk/DontWalk Light
		SysTick_Wait10ms(FSM[CurrState].Time);
		Input = SENSOR;													//read sensors
		CurrState = FSM[CurrState].Next[Input];	//Go to Next State
  }
}

void PortFBE_Init(void){
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x32;      // 1) F B E
  delay = SYSCTL_RCGC2_R;      // 2) // Allow time for clock to start
  GPIO_PORTE_AMSEL_R &= ~0x07; // 3) disable analog function on PE2-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x07;   // 5) inputs on PE2-0
  GPIO_PORTE_AFSEL_R &= ~0x07; // 6) regular function on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;    // 7) enable digital on PE2-0
	
  GPIO_PORTB_AMSEL_R &= ~0x3F; // 8) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 9) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 10) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // 11) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // 12) enable digital on PB5-0

	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 13) unlock GPIO Port F
	GPIO_PORTF_CR_R = 0x0A;           // 14)allow changes to PF3 and Pf1
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 15) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 16) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R |= 0x0A;          // 17)PF3 and PF1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 18) disable alt funct on PF7-0
  //GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0xA;          // 19) enable digital I/O on PF4-0
}

/*
//Program 10.1. Activate the LM4F/TM4C with a 16 MHz crystal to run at 80 MHz
void PLL_Init(void){
  // 0) Use RCC2
  SYSCTL_RCC2_R |=  0x80000000;  // USERCC2
  // 1) bypass PLL while initializing
  SYSCTL_RCC2_R |=  0x00000800;  // BYPASS2, PLL bypass
  // 2) select the crystal value and oscillator source
  SYSCTL_RCC_R = (SYSCTL_RCC_R &~0x000007C0)   // clear XTAL field, bits 10-6
                 + 0x00000540;   // 10101, configure for 16 MHz crystal
  SYSCTL_RCC2_R &= ~0x00000070;  // configure for main oscillator source
  // 3) activate PLL by clearing PWRDN
  SYSCTL_RCC2_R &= ~0x00002000;
  // 4) set the desired system divider
  SYSCTL_RCC2_R |= 0x40000000;   // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~ 0x1FC00000)  // clear system clock divider
                  + (4<<22);      // configure for 80 MHz clock
  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R&0x00000040)==0){};  // wait for PLLRIS bit
  // 6) enable use of PLL by clearing BYPASS
  SYSCTL_RCC2_R &= ~0x00000800;
}
*/

void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}

// 10000us equals 10ms
// 800000*12.5ns equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }
}


// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// green    -G-    0x08

// Kong Meng Vang
