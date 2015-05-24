/* Includes ------------------------------------------------------------------*/
#include "math.h"
#include "main.h"// redirect prinf to uart3, put GPS on interrupt
#include "stdbool.h"
#include <stdio.h>
#define ARRAYSIZE 8
#define ADC1_DR    ((uint32_t)0x4001244C)
#define AltK 1000
#define kV 100
#define kD 100
//volatile uint16_t ADC_values[ARRAYSIZE];

__IO uint16_t IC2Value = 0;
__IO uint16_t DutyCycle = 0;
__IO uint32_t Frequency = 0;
__IO uint16_t IC2Value1 = 0;
__IO uint16_t DutyCycle1 = 0;
__IO uint32_t Frequency1 = 0;
__IO uint32_t RT_Count=0;
__IO uint32_t SPIRXbuf;

__IO uint32_t deltaP;
__IO uint32_t lastP;
__IO uint32_t nowP;
__IO uint32_t baseP;
__IO uint32_t deltaA;
__IO uint32_t lastA;
__IO uint32_t nowA;
__IO uint32_t AltP;
__IO uint32_t AltA;
__IO uint32_t speed;
__IO uint32_t accl;
bool iGotNewToys=0;


//initiallize internal functions
void SPIInit(void);
void ADCInit(void);
void DMAInit(void);
void RCC_Configuration(void);
void Delay(__IO uint32_t nCount);
void GPIO_Configuration(void);
void Usart1Init(void);
void Usart1Put(uint8_t);
void TIM_Config(void);
uint8_t Usart1Get(void);

//initiallize sensors	
void initP(void);
void initT(void);
void initSD(void);
void initI(void);

//sensor actions
uint32_t getP(void);
uint32_t getTmp(void);
void getT(void);
void getIData(void);
void logSD(void);
void sendData(void);//rf send

uint32_t calcAlt(uint32_t,uint32_t);
void deploy(uint8_t);//main action function
uint8_t analyze(void);//main computation function

//list of componets:
//GPS-USART    latch data by interrupt(1 per second) NMEA, 4800 baud, 8 data bits, no parity, 1 stop bit, no flow control
//Xbee-USART
//ACC-SPI
//SD-SPI
//RTC-SPI
//Pressure-SPI  CPOL = 1, CPHA = 1 (SPI mode 3).
//RFPower

#define GPS_CS PAout(1)  //  Connect to push-pull output! This is mandatory!- Set to LOW by default- Toggle to HIGH and back to LOW
#define UART2_TX PAout(2)	
#define UART2_RX PAin(3) 

#define PS_CS PAout(4)	//for all SPI CS, low is valid
#define SPI_SCK PAout(5)	
#define SPI_MISO PAin(6)	
#define SPI_MOSI PAout(7)	
#define ACC_CS PAout(8)	
#define UART1_TX PAout(9)	
#define UART1_RX PAin(10)	
#define ACC_RST PAout(14) 
#define ACC_FLAG PAin(15)	

#define EMATCH1 PBout(6) 
#define EMATCH2 PBout(7)	
#define VR_CS PBout(8) 

#define USART3_TX PBout(10) 
#define USART3_RX PBin(11) 
#define RTC_CS PBout(12)	
#define USART3_CTS PBin(13)	
#define USART3_RTS PBout(14)	
#define SD_CS PBout(15)	



int main(void)
{
	//uint8_t index;
//	RCC_Configuration();
	EMATCH1=1;
	EMATCH2=1;
	GPIO_Configuration();
	Usart1Init();
	
	SPIInit();

	//DMAInit();
	TIM_Config();
	SysTick_Config(SystemCoreClock/10000);
	
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
  /* Enable SPIy */
  SPI_Cmd(SPI1, ENABLE);
	//Start ADC1 Software Conversion

	//print averages
	/*for(index = 0; index<8; index++)
			{
			printf("ch%d = %d ",index, ADC_values[index]);
			}*/
	
	initP();
	initT();
	initSD();
	initI();
	while (1)
	{
		RT_Count=0;
		getT();
		getIData();
		deltaP=lastP-nowP;
		deltaA=lastA-nowA;
		calcAlt(getP(),getTmp());// in ft
		logSD();
		speed=speed+(accl*RT_Count*kV); //riemann approx
		AltA=AltA+(speed*RT_Count*kD);		
		sendData();//send RF
		deploy(analyze());//1 will pop drogue chute, 2 will pop main, 0 will do nothing
	}
}

 
void Delay(__IO uint32_t num)
{
	__IO uint32_t index = 0;

	/* default system clock is 72MHz */
	for(index = (720 * num); index != 0; index--)
	{
	}
}



void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

void Usart1Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	 
	USART_InitTypeDef USART_InitStructure;
	 
	USART_ClockInitTypeDef USART_ClockInitStructure;
	 
	//enable bus clocks
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	 
	//Set USART1 Tx (PA.09) as AF push-pull
	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	//Set USART1 Rx (PA.10) as input floating
	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	USART_ClockStructInit(&USART_ClockInitStructure);
	 
	USART_ClockInit(USART1, &USART_ClockInitStructure);
	 
	USART_InitStructure.USART_BaudRate = 256000;
	 
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	 
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	 
	USART_InitStructure.USART_Parity = USART_Parity_Even ;
	 
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	 
	//Write USART1 parameters
	 
	USART_Init(USART1, &USART_InitStructure);
	 
	//Enable USART1
	 
	USART_Cmd(USART1, ENABLE);
	USART_ClockStructInit(&USART_ClockInitStructure);
	USART_ClockInit(USART1, &USART_ClockInitStructure);
 
}


void SPIInit()
{
	SPI_InitTypeDef   SPI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
       
  /* System clocks configuration ---------------------------------------------*/
 
  
  /* SPIy Config -------------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//1
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;

  SPI_Init(SPI1, &SPI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	


}
void SPIsend(uint32_t buf)
{
		//wait for buffer to empty up
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    /* Send SPIz data */
    SPI_I2S_SendData(SPI1, buf);
}
void SPI1_IRQHandler(void)
{
    /* Check the interrupt source */
    /* RX */
  if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) == SET)
  {
		SPIRXbuf=SPI_I2S_ReceiveData(SPI2);
	//load shits to a table or some shit
  }
	iGotNewToys=1;//software flag for waiting conditions
}
void TIM_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;

//	GPIO_InitTypeDef GPIO_InitStructure1;
  NVIC_InitTypeDef NVIC_InitStructure1;
//	TIM_ICInitTypeDef  TIM_ICInitStructure1;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);


  /* NVIC configuration */
//  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the TIM4 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure1.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure1.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure1.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure1.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure1);


	
  /* Configure the GPIO ports */


  /* TIM4 channel 2 pin (PA.07) configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);



  /* TIM4 configuration: PWM Input mode ------------------------
     The external signal is connected to TIM4 CH2 pin (PA.01), 
     The Rising edge is used as active edge,
     The TIM4 CCR2 is used to compute the frequency value 
     The TIM4 CCR1 is used to compute the duty cycle value
  ------------------------------------------------------------ */

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
	
  TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);
	TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

  /* Select the TIM4 Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);
  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
  /* Enable the Master/Slave Mode */
  TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
  /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE); 
}

void TIM4_IRQHandler(void)
{
  /* Clear TIM4 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);

  /* Get the Input Capture value */
  IC2Value = TIM_GetCapture2(TIM4);

  if (IC2Value != 0)
  {
    /* Duty cycle computation */
    DutyCycle = (TIM_GetCapture1(TIM4) * 100) / IC2Value;

    /* Frequency computation */
    Frequency = SystemCoreClock / IC2Value;
//		printf("%d/n",Frequency);
  }
  else
  {
    DutyCycle = 0;
    Frequency = 0;
  }
}
void TIM2_IRQHandler(void)
{
  /* Clear TIM4 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

  /* Get the Input Capture value */
  IC2Value1 = TIM_GetCapture2(TIM2);

  if (IC2Value1 != 0)
  {
    /* Duty cycle computation */
    DutyCycle1 = (TIM_GetCapture1(TIM2) * 100) / IC2Value1;

    /* Frequency computation */
    Frequency1 = SystemCoreClock / IC2Value1;
//		printf("%d/n",Frequency);
  }
  else
  {
    DutyCycle1 = 0;
    Frequency1 = 0;
  }
}
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	RT_Count++;
}

uint8_t analyze(void)
{
	return 1;
}
void deploy(uint8_t action)
{
	if(action==1)
		EMATCH1=1;
	else if(action==2)
		EMATCH2=1;
}
void initP()
{
}
void initT()
{
}
void initSD()
{
}
void initI()
{
}
void getP(void)// pressure sensor is on SPI
{
}
void getT(void)// RTC is on SPI
{
}
void getIData(void)
{
}
void logSD(void)
{
}
void sendData(void)
{
}
