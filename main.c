#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#define ENABLE_PIN 2 //Enable Pin PA2
#define MOTOR_INPUT1_PIN 3 //CW Rotation Pin PA3
#define MOTOR_INPUT2_PIN 4 //CCW Rotation Pin PA4
void PortF_Init(void);
void PortA_Init(void);
void PortE_Init(void);
void PortB_Init(void);
void PortD_Init(void);


static void Jamming (void *pvParameters);
static void Up_driver (void *pvParameters);
static void Up_passenger (void *pvParameters);
static void Down_passenger (void *pvParameters);
static void Down_driver (void *pvParameters);
static void Lock (void *pvParameters);




void delay_ms(int n);
//define a task handle
xTaskHandle xJamming;
xTaskHandle xUp_driver;
xTaskHandle xUp_passenger;
xTaskHandle xDown_passenger;
xTaskHandle xDown_driver;
//defining Semaphore Handles
xSemaphoreHandle xJamming_Semaphore;
xSemaphoreHandle xUp_driver_Semaphore;
xSemaphoreHandle xUp_passenger_Semaphore;
xSemaphoreHandle xDown_passenger_Semaphore;
xSemaphoreHandle xDown_driver_Semaphore;
xSemaphoreHandle xLock_Semaphore;




 //                   /main function
//------------------------------------------------------------------------//
int main( void )
{
	//Initializing all Ports
    PortF_Init();
	  PortA_Init();
	  PortB_Init();
	  PortD_Init();
	  PortE_Init();
   //Enable Interrupts through Processor
		__ASM("CPSIE i");
		//Semaphores Creation 
		vSemaphoreCreateBinary(xJamming_Semaphore);
	  vSemaphoreCreateBinary(xUp_driver_Semaphore);
		vSemaphoreCreateBinary(xUp_passenger_Semaphore);
		vSemaphoreCreateBinary(xDown_passenger_Semaphore);
		vSemaphoreCreateBinary(xDown_driver_Semaphore);
		vSemaphoreCreateBinary(xLock_Semaphore);


	if( (xJamming_Semaphore  && xUp_driver_Semaphore && xUp_passenger_Semaphore && xDown_passenger_Semaphore && xDown_driver_Semaphore && xLock_Semaphore )!= NULL)
		{
			/* Create the 'handler' task. This is the task that will be synchronized
			with the interrupt. The handler task is created with a high priority to
			ensure it runs immediately after the interrupt exits. In this case a
			priority of 3 is chosen. */
			xTaskCreate( Jamming, "Move Up Task", 240, NULL, 5, &xJamming );
    	xTaskCreate( Up_driver , "Move auto Task", 240, NULL, 4, &xUp_driver );
			xTaskCreate( Up_passenger , "Move auto Task", 240, NULL, 2, &xUp_passenger );
			xTaskCreate( Down_passenger , "Move auto Task", 240, NULL, 2, &xDown_passenger );
			xTaskCreate( Down_driver , "Move auto Task", 240, NULL, 4, &xDown_driver );
			xTaskCreate( Lock , "Lock Task", 240, NULL, 2, NULL );


			/* Create the task that will periodically generate a software interrupt.
			This is created with a priority below the handler task to ensure it will
			get preempted each time the handler task exits the Blocked state. */
			//xTaskCreate( vPeriodicTask, "Periodic", 240, NULL, 1, NULL );
			/* Start the scheduler so the created tasks start executing. */
			vTaskStartScheduler();
		}

    /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
    for( ;; );
}

//------------------------------------------------------------------------/
//Initialize the hardware of Port-A
void PortA_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000001;    // 1) A clock
  GPIOA->LOCK = 0x4C4F434B;  				 // 2) unlock PortA  
  GPIOA->CR = 0xFF;          				 // allow changes to PortA       
  GPIOA->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOA->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOA->DIR = 0x1C;         				 // 5) PA0,PA1 input, PA2,PA3,PA4 output   
  GPIOA->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOA->PUR = 0xE3;       				   // enable pullup resistors      
  GPIOA->DEN = 0xFF;       				   // 7) enable digital pins 
	GPIOA->DATA = 0x00;
	
	// Setup the interrupt on PortA
	GPIOA->ICR = 0xFF;     // Clear any Previous Interrupt 
	GPIOA->IM |=0x01;      // Unmask the interrupts for PA0 
	GPIOA->IS |= ~(0x01);     // Make bits PA0  level sensitive
	GPIOA->IEV &= ~(0x01);   // Sense on Low Level
  
	NVIC_EnableIRQ(0);        // Enable the Interrupt for PortA in NVIC
}


void PortB_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000002;    // 1) B clock
  GPIOB->LOCK = 0x4C4F434B;  				 // 2) unlock PortB  
  GPIOB->CR = 0xFF;          				 // allow changes to PortB       
  GPIOB->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOB->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOB->DIR = 0x00;         				 // 5) all inputs
  GPIOB->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOB->PUR = 0xFF;       				   // enable pullup resistors       
  GPIOB->DEN = 0xFF;       				   // 7) enable digital pins 
	GPIOB->DATA = 0x00;
	
	// Setup the interrupt on PortB
	GPIOB->ICR = (1<<0) | (1<<1);     // Clear any Previous Interrupt 
	GPIOB->IM |= (1<<0) | (1<<1);      // Unmask the interrupts for PB0 and PB1
	GPIOB->IS &= ~((1<<0) | (1<<1));     // Make bits PB0 and PB1 level sensitive
	GPIOB->IEV &= ~((1<<0) | (1<<1));   // Sense on Low Level

	NVIC_EnableIRQ(1);        // Enable the Interrupt for PortB in NVIC
}

void PortC_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000004;    // 1) C clock
  GPIOC->LOCK = 0x4C4F434B;  				 // 2) unlock PortC   
  GPIOC->CR = 0xE0;          				 // allow changes to PortC       
  GPIOC->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOC->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOC->DIR = 0x00;         				 // 5) all inputs  
  GPIOC->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOC->PUR = 0xE0;       				   // enable pullup resistors on PC5,PC6,PC7      
  GPIOC->DEN = 0xE0;       				   // 7) enable digital pins PC5,PC6,PC7
	GPIOC->DATA = 0x00;
	
	// Setup the interrupt on PortC
	GPIOC->ICR = 0xFF;     // Clear any Previous Interrupt 
	GPIOC->IM |=0x80;      // Unmask the interrupts for PC7
	GPIOC->IS &= ~(0x80);     // Make bits PC7 level sensitive
	GPIOC->IEV &= ~(0x80);   // Sense on Low Level

	NVIC_EnableIRQ(2);        // Enable the Interrupt for PortC in NVIC
}

void PortD_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000008;    // 1) D clock
  GPIOD->LOCK = 0x4C4F434B;  				 // 2) unlock PortD 
  GPIOD->CR = 0x1F;          				 // allow changes to PortD      
  GPIOD->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOD->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOD->DIR = 0x00;         				 // 5) all inputs 
  GPIOD->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOD->PUR = 0xFF;       				   // enable pullup resistors       
  GPIOD->DEN = 0xFF;       				   // 7) enable digital pins 
	GPIOD->DATA = 0x00;
	
	// Setup the interrupt on PortD
	GPIOD->ICR = (1<<0) | (1<<1);     // Clear any Previous Interrupt 
	GPIOD->IM |= (1<<0) | (1<<1);      // Unmask the interrupts for PD0 and PD1
	GPIOD->IS &= ~((1<<0) | (1<<1));     // Make bits PD0 and PD1 level sensitive
	GPIOD->IEV &= ~((1<<0) | (1<<1));   // Sense on Low Level

	NVIC_EnableIRQ(3);        // Enable the Interrupt for PortD in NVIC
}


void PortE_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000010;    // 1) E clock
  GPIOE->LOCK = 0x4C4F434B;  				 // 2) unlock PortE 
  GPIOE->CR = 0x1F;          				 // allow changes to PortE      
  GPIOE->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOE->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOE->DIR = 0x0C;         				 // 5) PE2,PE3 output 
  GPIOE->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOE->PUR = (1<<0) | (1<<1);       				   // enable pullup resistors on PE0,PE1       
  GPIOE->DEN = 0x1F;       				   // 7) enable digital pins 
	GPIOE->DATA = 0x00;
	
	// Setup the interrupt on PortE
	GPIOE->ICR = (1<<0) | (1<<1);     // Clear any Previous Interrupt 
	GPIOE->IM |= (1<<0) | (1<<1);      // Unmask the interrupts for PE0,PE1
	GPIOE->IS &= ~((1<<0) | (1<<1));     // Make bits PE0, PE1 level sensitive
	GPIOE->IEV &= ~((1<<0) | (1<<1));   // Sense on Low Level

	NVIC_EnableIRQ(4);        // Enable the Interrupt for PortE in NVIC
}

void PortF_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000020;    // 1) F clock
  GPIOF->LOCK = 0x4C4F434B;  				 // 2) unlock PortF PF0  
  GPIOF->CR = 0x1F;          				 // allow changes to PF4-0       
  GPIOF->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOF->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOF->DIR = 0x00;         				 // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIOF->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOF->PUR = 0x1F;       				   // enable pullup resistors on PF4,PF0       
  GPIOF->DEN = 0x1F;       				   // 7) enable digital pins PF4-PF0
	GPIOF->DATA = 0x00;
	
	// Setup the interrupt on PortF
	GPIOF->ICR = 0x1F;   // Clear any Previous Interrupt 
	GPIOF->IM |=(1<<0) | (1<<1);   // Unmask the interrupts for PF0 and PF4/
	GPIOF->IS &= ~((1<<0) | (1<<1));    // Make bits PF0 and PF4 level sensitive
	GPIOF->IEV &= ~((1<<0) | (1<<1));    // Sense on Low Level

	NVIC_EnableIRQ(30);        // Enable the Interrupt for PortF in NVIC
  NVIC_SetPriority( 30, 0xe0 );
	
}
static void Jamming (void *pvParameters)
{
	//Block the Jamming Task for the first time only
	xSemaphoreTake(xJamming_Semaphore,0);
	while(1)
	{
	//Unblock the Jamming task 
	 xSemaphoreTake(xJamming_Semaphore,portMAX_DELAY);
			int ticks=0;
		
		// wait for 0.5 seconds delay before motor disable
    while (ticks < (80*3180)){		
    GPIOA->DATA &= ~(1 << MOTOR_INPUT2_PIN);
    GPIOA->DATA |= (1 << MOTOR_INPUT1_PIN); 
		GPIOA->DATA |= (1<<ENABLE_PIN);		
			ticks++;

}
		//Disable Motor
   GPIOA->DATA &= ~(1 << ENABLE_PIN );
//Suspend Driver and Passenger tasks 
   vTaskSuspend(xUp_driver);
   vTaskSuspend(xUp_passenger);
}
	

	}

static void Down_passenger (void *pvParameters)
	{
		//Block the Down Passenger Task for the first time only
		xSemaphoreTake(xDown_passenger_Semaphore,0);
	while(1)
	{
		//Unblock the task by taking semaphore
	  xSemaphoreTake(xDown_passenger_Semaphore,portMAX_DELAY);
		
		int ticks = 0 ; 
		//Check the Manual Condition taking into consideration the pressing time of the button and the asscoiated limit switch and the lock switch
		if ((GPIOD->DATA & (1 << 1)) == 0 && ((GPIOE->DATA & (1<<0)) != 0) && ((GPIOF->DATA & (1<<1)) != 0)){
			while(ticks < (150*3180)){
		GPIOA->DATA |= (1 << MOTOR_INPUT1_PIN);
    GPIOA->DATA &= ~(1 << MOTOR_INPUT2_PIN); 
		GPIOA->DATA |= (1<<ENABLE_PIN);		
			ticks++;
}
			}
				//Check the Automatic Condition taking into consideration the pressing time of the button and the asscoiated limit switch and the lock switch

			while ((GPIOD->DATA & (1 << 1)) == 0 && ((GPIOE->DATA & (1<<0)) != 0)&&((GPIOF->DATA & (1<<1)) != 0) ){
				if ((GPIOD->DATA & (1 << 1)) == 0 && ((GPIOE->DATA & (1<<0)) != 0)&&((GPIOF->DATA & (1<<1)) != 0)){
		GPIOA->DATA |= (1 << MOTOR_INPUT1_PIN);
    GPIOA->DATA &= ~(1 << MOTOR_INPUT2_PIN); 
		GPIOA->DATA |= (1<<ENABLE_PIN);	

			}
     GPIOA->DATA &= ~ (1<<ENABLE_PIN);		

		}
	vTaskResume(xUp_driver);
  vTaskResume(xUp_passenger);
	}
	
		
	}

static void Up_driver (void *pvParameters)
{
	//PB2,PB3
			//Block the Up Driver Task for the first time only

	xSemaphoreTake(xUp_driver_Semaphore,0);
	while(1)
	{
				//Unblock the task by taking semaphore

	  xSemaphoreTake(xUp_driver_Semaphore,portMAX_DELAY);
		
		int ticks = 0 ; 
				//Check the Manual Condition taking into consideration the pressing time of the button and the asscoiated limit switch 

		if ((GPIOB->DATA & (1 << 0)) == 0 && ((GPIOE->DATA & (1<<1)) != 0)){
			while(ticks < (150*3180)){
		GPIOA->DATA |= (1 << MOTOR_INPUT2_PIN);
    GPIOA->DATA &= ~(1 << MOTOR_INPUT1_PIN); 
		GPIOA->DATA |= (1<<ENABLE_PIN);		
			ticks++;
}
			}
				//Check the Automatic Condition taking into consideration the pressing time of the button and the asscoiated limit switch 

			while ((GPIOB->DATA & (1 << 0)) == 0 && ((GPIOE->DATA & (1<<1)) != 0)){
				if ((GPIOB->DATA & (1 << 0)) == 0 && ((GPIOE->DATA & (1<<1)) != 0) ){
		GPIOA->DATA |= (1 << MOTOR_INPUT2_PIN);
    GPIOA->DATA &= ~(1 << MOTOR_INPUT1_PIN); 
		GPIOA->DATA |= (1<<ENABLE_PIN);	
			}
			GPIOA->DATA &= ~ (1<<ENABLE_PIN);		
		}
	}
}

static void Up_passenger (void *pvParameters)
{
	//PB2,PB3
				//Block the Up Passenger Task for the first time only

	xSemaphoreTake(xUp_passenger_Semaphore,0);
	while(1)
	{
						//Unblock the task by taking semaphore

	  xSemaphoreTake(xUp_passenger_Semaphore,portMAX_DELAY);
		
		int ticks = 0 ; 
						//Check the Manual Condition taking into consideration the pressing time of the button and the asscoiated limit switch  and the lock switch

		if ((GPIOD->DATA & (1 << 0)) == 0 && ((GPIOE->DATA & (1<<1)) != 0)&&((GPIOF->DATA & (1<<1)) != 0)){
			while(ticks < (150*3180)){
		GPIOA->DATA |= (1 << MOTOR_INPUT2_PIN);
    GPIOA->DATA &= ~(1 << MOTOR_INPUT1_PIN); 
		GPIOA->DATA |= (1<<ENABLE_PIN);		
			ticks++;
}
			}
					//Check the Automatic Condition taking into consideration the pressing time of the button and the asscoiated limit switch  and the lock switch

			while ((GPIOD->DATA & (1 << 0)) == 0 && ((GPIOE->DATA & (1<<1)) != 0)&&((GPIOF->DATA & (1<<1)) != 0)){
				if ((GPIOD->DATA & (1 << 0)) == 0 && ((GPIOE->DATA & (1<<1)) != 0)&&((GPIOF->DATA & (1<<1)) != 0)){
		GPIOA->DATA |= (1 << MOTOR_INPUT2_PIN);
    GPIOA->DATA &= ~(1 << MOTOR_INPUT1_PIN); 
		GPIOA->DATA |= (1<<ENABLE_PIN);	

			}
     GPIOA->DATA &= ~ (1<<ENABLE_PIN);		

		}
	}
}



static void Down_driver (void *pvParameters)
{
					//Block the Down Driver Task for the first time only

			xSemaphoreTake(xDown_driver_Semaphore,0);
	while(1)
	{
								//Unblock the task by taking semaphore

	  xSemaphoreTake(xDown_driver_Semaphore,portMAX_DELAY);
		
		int ticks = 0 ; 
			//Check the Manual Condition taking into consideration the pressing time of the button and the asscoiated limit switch  

		if ((GPIOB->DATA & (1 << 1)) == 0 && ((GPIOE->DATA & (1<<0)) != 0)){
			while(ticks < (150*3180)){
		GPIOA->DATA |= (1 << MOTOR_INPUT1_PIN);
    GPIOA->DATA &= ~(1 << MOTOR_INPUT2_PIN); 
		GPIOA->DATA |= (1<<ENABLE_PIN);		
			ticks++;
}
			}
		//Check the Manual Condition taking into consideration the pressing time of the button and the asscoiated limit switch  

			while ((GPIOB->DATA & (1 << 1)) == 0 && ((GPIOE->DATA & (1<<0)) != 0) ){
				if ((GPIOB->DATA & (1 << 1)) == 0 && ((GPIOE->DATA & (1<<0)) != 0)){
		GPIOA->DATA |= (1 << MOTOR_INPUT1_PIN);
    GPIOA->DATA &= ~(1 << MOTOR_INPUT2_PIN); 
		GPIOA->DATA |= (1<<ENABLE_PIN);	

			}
     GPIOA->DATA &= ~ (1<<ENABLE_PIN);	
    
		}
			vTaskResume(xUp_driver);
    vTaskResume(xUp_passenger);			
   
	}
		
	
	
	
}
static void Lock (void *pvParameters)
{
						//Block the Down Driver Task for the first time only

	xSemaphoreTake(xLock_Semaphore,0);
	while(1)
	{
										//Unblock the task by taking semaphore

	  xSemaphoreTake(xLock_Semaphore,portMAX_DELAY);
		//Disable the Motor while the Lock switch button is pressed
		while (GPIOF->DATA && (1<<1) != 1)
		{
			GPIOA->DATA &= ~ (1<<ENABLE_PIN);	
		}
	}
	
}

	
void GPIOF_Handler(void){

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	delay_ms(30);
	if (GPIOF->MIS & (1<<0))
	{
	//Give the semaphore to the Task named handler
  xSemaphoreGiveFromISR(xJamming_Semaphore,&xHigherPriorityTaskWoken);
	//mainCLEAR_INTERRUPT();
	GPIOF->ICR = 0x01;        // clear the interrupt flag of PORTF
  i= GPIOF->ICR ;           // Reading the register to force the flag to be cleared
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	}
	if (GPIOF->MIS & (1<<1))
	{
	//Give the semaphore to the Task named handler
  xSemaphoreGiveFromISR(xLock_Semaphore,&xHigherPriorityTaskWoken);
	//mainCLEAR_INTERRUPT();
	GPIOF->ICR = 0xFF;        // clear the interrupt flag of PORTF
  i= GPIOF->ICR ;           // Reading the register to force the flag to be cleared
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	}
	
	
}

void GPIOB_Handler(void){

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	delay_ms(30);
	if (GPIOB->MIS & (1<<0))
	{
	//Give the semaphore to the Task named handler
  xSemaphoreGiveFromISR(xUp_driver_Semaphore,&xHigherPriorityTaskWoken);
	//mainCLEAR_INTERRUPT();
	GPIOB->ICR = 0x01;        // clear the interrupt flag of PORTB
  i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	}
	if (GPIOB->MIS & (1<<1))
	{
	//Give the semaphore to the Task named handler
  xSemaphoreGiveFromISR(xDown_driver_Semaphore,&xHigherPriorityTaskWoken);
	//mainCLEAR_INTERRUPT();
	GPIOB->ICR = (1<<1);        // clear the interrupt flag of PORTB
  i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	}
	
}


void GPIOD_Handler(void){

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	delay_ms(30);
	if (GPIOD->MIS & (1<<0))
	{
	//Give the semaphore to the Task named handler
  xSemaphoreGiveFromISR(xUp_passenger_Semaphore,&xHigherPriorityTaskWoken);
	//mainCLEAR_INTERRUPT();
	GPIOD->ICR = 0x01;        // clear the interrupt flag of PORTD
  i= GPIOD->ICR ;           // Reading the register to force the flag to be cleared
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	}
	if (GPIOD->MIS & (1<<1))
	{
	//Give the semaphore to the Task named handler
  xSemaphoreGiveFromISR(xDown_passenger_Semaphore,&xHigherPriorityTaskWoken);
	//mainCLEAR_INTERRUPT();
	GPIOD->ICR = (1<<1);        // clear the interrupt flag of PORTD
  i= GPIOD->ICR ;           // Reading the register to force the flag to be cleared
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	}
	
}


//Port-E handler

void GPIOE_Handler(void){

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	delay_ms(30);
	//Give the semaphore to the Task named handler
 // xSemaphoreGiveFromISR(xLimit_switch_Semaphore,&xHigherPriorityTaskWoken);
		if (GPIOE->MIS & (1<<1)){
	//mainCLEAR_INTERRUPT();
	GPIOE->ICR = (1<<1);        // clear the interrupt flag of PORTE
  i= GPIOE->ICR ;           // Reading the register to force the flag to be cleared
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
					GPIOA->DATA &= ~(1 << ENABLE_PIN );

		}
		if (GPIOE->MIS & (1<<0)){
	//mainCLEAR_INTERRUPT();
	GPIOE->ICR = (1<<0);        // clear the interrupt flag of PORTE
  i= GPIOE->ICR ;           // Reading the register to force the flag to be cleared
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
					GPIOA->DATA &= ~(1 << ENABLE_PIN );

	
			
		}
	

}
//Crude Delay Implementation
void delay_ms(int n)
{
 int i,j;
 for(i=0;i<n;i++)
 for(j=0;j<3180;j++)
 {}
}