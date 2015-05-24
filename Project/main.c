/* Includes ------------------------------------------------------------------*/
#include "math.h"
#include "main.h"// redirect prinf to uart3, put GPS on interrupt
#include "stdbool.h"
#include <stdio.h>
#define ARRAYSIZE 8
#define ADC1_DR    ((uint32_t)0x4001244C)
#define AltK 1000
#define MainDeployHight 3000
#define kV 0.5   //speed integration tuning coeff
#define kD 0.3	//distance integration tuning coeff
#define oss 1   //over sampling_ seeting 0 ultra low-p 1 std 2 hres 3uhres
//volatile uint16_t ADC_values[ARRAYSIZE];

__IO uint16_t IC2Value = 0;
__IO uint16_t DutyCycle = 0;
__IO uint32_t Frequency = 0;
__IO uint16_t IC2Value1 = 0;
__IO uint16_t DutyCycle1 = 0;
__IO uint32_t Frequency1 = 0;
__IO uint32_t RT_Count=0;
__IO uint32_t SPIRXbuf;
__IO uint8_t CaliData[22];

__IO uint32_t deltaP;
__IO uint32_t lastP;
__IO uint32_t nowP;
__IO uint32_t baseP;
__IO uint32_t deltaA;
__IO uint32_t lastA;
__IO uint32_t nowA;
__IO uint32_t AltP;
__IO uint32_t AltA;
__IO int32_t speed;
__IO uint32_t accl;

//calibration data with initial values
short	B1=6190;
short	B2=4;
short	AC1=408;
short	AC2=-72;
short	AC3=-14383;
unsigned short AC4=32741;
unsigned short AC5=32757;
unsigned short AC6=23153;
short MC=-8711;
short MD=2868;
	
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
uint16_t getP(void);
uint32_t getTmp(void);
void getT(void);
void getIData(void);
void logSD(void);
void sendData(void);//rf send

int32_t calcAlt(uint16_t,uint32_t);
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
		AltP=calcAlt(getP(),getTmp());// in ft
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
	int16_t deltaAlt=0;
	
	if(deltaAlt<0 && speed<0)
		return 1;
	else if(AltP<MainDeployHight)
		return 2;
	else
		return 0;
	
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
	uint8_t i;
	PS_CS=0;
	SPIsend(0x2a);//point at 0xAA
	for(i=0;i<21;i++)
	{
	while(!iGotNewToys);
	iGotNewToys=0;
	CaliData[i]=SPIRXbuf;
	}
	PS_CS=1;
	
	AC1=(CaliData[0]<<8)+CaliData[1];
	AC2=(CaliData[2]<<8)+CaliData[3];
	AC3=(CaliData[4]<<8)+CaliData[5];
	AC4=(CaliData[6]<<8)+CaliData[7];
	AC5=(CaliData[8]<<8)+CaliData[9];
	AC6=(CaliData[10]<<8)+CaliData[11];
	B1=(CaliData[12]<<8)+CaliData[13];
	B2=(CaliData[14]<<8)+CaliData[15];
	MC=(CaliData[18]<<8)+CaliData[19];
	MD=(CaliData[20]<<8)+CaliData[21];
	
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
uint16_t getP(void)// pressure sensor is on SPI
{
	uint16_t results;
	uint8_t LR,HR;
	
	PS_CS=0;
	SPIsend(0x74);
	SPIsend(0x2e);
	PS_CS=1;
	
	Delay(80);//wait 4.5ms
	
	PS_CS=0;
	SPIsend(0xf6);
	while(!iGotNewToys);
	iGotNewToys=0;
	HR=SPIRXbuf;
	while(!iGotNewToys);
	iGotNewToys=0;
	LR=SPIRXbuf;
	PS_CS=1;
	
	results=(HR<<8)+LR;
	return results;
}
uint32_t getTmp(void)// pressure sensor is on SPI
{
	uint32_t results;
	uint8_t XLR,LR,HR;
	
	PS_CS=0;
	SPIsend(0x74);
	SPIsend(0x34<<(oss+6));
	PS_CS=1;
	
	Delay(80);//wait 4.5ms
	
	PS_CS=0;
	SPIsend(0xf6);
	while(!iGotNewToys);
	iGotNewToys=0;
	HR=SPIRXbuf;
	while(!iGotNewToys);
	iGotNewToys=0;
	LR=SPIRXbuf;
	while(!iGotNewToys);
	iGotNewToys=0;
	XLR=SPIRXbuf;
	PS_CS=1;
	
	results=((HR<<16)+(LR<<8)+XLR)>>(8-oss);
	return results;
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
int32_t calcAlt(uint16_t UP,uint32_t UT)
{
	long X1,X2,X3,B5,Temprature,B3,B6,B7,p;
	unsigned long B4;
	X1=(UT*AC6)*AC5/2^15;
	X2=MC*2^11/(X1+MD);
	B5=X1+X2;
	Temprature=(B5+8)/2^4;
	
	B6=B5*4000;
	X1=(B2*(B6*B6/2^12))/2^11;
	X2=AC2*B6/2^11;
	X3=X1+X2;
	B3=(((AC1*4+X3)<<oss) +2)/4;
	X1=AC3*B6/2^13;
	X2=(B1*(B6*B6/2^12))/2^16;
	X3=((X1+X2)+2)/2^2;
	B4=AC4*(unsigned long)(X3+32768)/2^15;
	B7=((unsigned long)UP-B3)*(50000>>oss);
	if(B7<0x80000000)
		p=(B7*2)/B4;
	else
		p=(B7/B4)*2;
	X1=(p/2^8)*(p/2^8);
	X1=(X1*3038)/2^16;
	X2=(-7357*p)/2^16;
	p=p+(X1+X2+3791)/2^4;
	int32_t results;
	return results;
}
