
/**
  ******************************************************************************

  Control DC Motor speed with PID algorithme based at STM32F407VG board
  Author:   Mouadh Dahech
  Updated:  3/21/2021
	
	********************************* Wiring *************************************
	* L298N:
	- IN1 -> PB0
	- IN2 -> PB1
	- ENA -> PD12
	* MOTOR ENCODER :
	- RED   -> OUT1
	- BLACK -> OUT2
	- GREEN -> GND
	- BLUE  -> 5V
	- WHITE -> PA0
	
  ******************************************************************************
  Copyright (C) 

  ******************************************************************************
*/
#include "stm32f407xx.h"
#include <stdbool.h> 

/***************************** Functions Prototypes *********************************/
void SystemClock(void);
void PWM_Config();
void Tim6Config();
void Tim7Config();
void GPIO_config();
uint32_t millis(void);
void msdelay(uint16_t ms);
void EXTI_config();
/****************************** Global macros variables and constant*****************/
#define encoderoutput  211  
#define motorSpeed TIM4->CCR1 
#define interval 1000
#define Speed  100
/****************************** Global Varibales and constant ************************************/
unsigned long previousMillis = 0;
unsigned long currentMillis  = 0;
unsigned long previoustime   = 0;
unsigned int rpm = 0;
unsigned long ticks = 0 ;
volatile long encoderValue = 0;
const float kp = 2;
const float kd = 0.0;
const float ki = 0.0;
unsigned int goal = 300;
int error;
int previouserror = 0;
float deltaT;
float deriv = 0;
float integral = 0;
int adjustement;
bool state = 0;

/****************************** main function **************************************/
int main(void)
{
  Tim7Config();
  PWM_Config();
  EXTI_config();
  GPIO_config();
  while (1)
  {  
		motorSpeed = 300;	
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) 
    {
      previousMillis = currentMillis;
      rpm = (float)(encoderValue * 60 / encoderoutput);
      encoderValue = 0;
    }
		
		
 
  }

  return 0 ; 
}

/********************* this function config PWM signal to control Dc motor speed: PWM-->PD12***********/  
void PWM_Config()
{
  //1. active the clock 
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN ;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  //2. GPIOD PIN_12 Alternate function 
  GPIOD->MODER = 0 ;
  GPIOD->MODER |= (1<<25);

  //3.AF TIM4
  GPIOD->AFR[1] = 0x00020000;    

  // Enable channel 1 compare register
  TIM4->CCER |= (1<<0);
  TIM4->CR1  |= TIM_CR1_ARPE;

  //4. PWM Mode 1 
  TIM4->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |  TIM_CCMR1_OC1PE;

  // 50 hz frq          
  TIM4->PSC  = 320;       //16000000/320*1000 = 50 hz
  TIM4->ARR  = 1000;     

  //duty cycle: 0--1000; 
  TIM4->CCR1 = 0;   // 0 degree
  
  TIM4->EGR |= TIM_EGR_UG;
  TIM4->CR1 |= TIM_CR1_CEN;

}

/********************************* Function to config interrupts at line 0: PA0****************/
void EXTI_config()
{
  // Enable clock for GPIOA: EXTI-->PA0
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  // Pull down 
  GPIOA->PUPDR &= !3U << 0;
  GPIOA->PUPDR |= 2U << 0;

  // Enable interrupt (cmsis)
  NVIC_EnableIRQ(EXTI0_IRQn);

 // Enable syscfg clock 
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

 // Enable line 0 interupt
  SYSCFG->EXTICR[0] =(0x0 << 0);

 // Set interrupt Mask register  
  EXTI->IMR |= EXTI_IMR_IM0;

  // Config interrupt every rsing edge 
  EXTI->RTSR |= EXTI_RTSR_TR0;
	EXTI->FTSR |= EXTI_FTSR_TR0;
}

/************************ This function to config GPIO Pins ************************/
void GPIO_config()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  GPIOB->MODER = 0x5;
  GPIOB->ODR = 0x0002;
}

/*************************** this function call every EXTI0 interrupt**********************/
void EXTI0_IRQHandler(void)
{
  if((EXTI->PR & EXTI_PR_PR0) != 0)
  {
    encoderValue++;
    EXTI->PR |= EXTI_PR_PR0;
  }
}


 uint32_t systemMillis = 0 ;
/*********************** Config Timer 6 to use in us and ms delay********************/
void Tim6Config()
{
 //1. Enable the clock for TIM6
  RCC->APB1ENR |= (1<<4);

 //2. Set the prescaler to 15 to run the counter with 1MHZ freq so 1 us for each tick
  TIM6->PSC = 15;

 //3. Max value 
  TIM6->ARR = 0xffff;

 //4. Active counter up 
  TIM6->CR1 |= (1<<0);
  while(!(TIM6->SR & (1<<0)));
}

/*********************** Config Timer 7 to create mills function********************/
void Tim7Config()
{
 //1. Enable the clock for TIM7
  RCC->APB1ENR |= (1<<5);

 //2. Set the prescaler to 159 to run the counter with 100kHZ freq so 1 ms for 100 ticks
  TIM7->PSC = 159;

 //3. Max value 
  TIM7->ARR = 0xffff;

 //4. Active counter up 
  TIM7->CR1 |= (1<<0);
  while(!(TIM7->SR & (1<<0)));
}

/*********************** this function to return time in millis ********************/
uint32_t millis(void)
{
  uint16_t TIM7CNT = TIM7->CNT;
  uint16_t currentMills = 0;
  if(TIM7CNT >= 100)
    {
      currentMills = TIM7CNT/100;
      systemMillis = systemMillis + currentMills;
      TIM7->CNT = 0;
    }
  return systemMillis;
}

void usdelay(uint16_t us)
{
 TIM6->CNT = 0;
 while(TIM6->CNT < us);
}

void msdelay(uint16_t ms)
{
  for(uint16_t i = 0; i<ms; i++)
  {
    usdelay(1000);
  }  
}