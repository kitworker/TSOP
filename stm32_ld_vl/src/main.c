///**
//  ******************************************************************************
//  * @file    RC5_Receiver_Demo/src/main.c
//  * @author  MCD Application Team
//  * @version V1.0.0
//  * @date    03/16/2010
//  * @brief   Main program body
//  ******************************************************************************
//  * @copy
//  *
//  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
//  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
//  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
//  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
//  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
//  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//  *
//  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
//  */
//
///* Includes ------------------------------------------------------------------*/
//#include "stm32f10x.h"
//#include "stm32_eval.h"
//#include "RC5_IR_Emul_Receiver.h"
//#include <stdio.h>
//
///** @addtogroup STM32F10x_StdPeriph_Examples
//  * @{
//  */
//
///* Private typedef -----------------------------------------------------------*/
///* Private define ------------------------------------------------------------*/
///* Private macro -------------------------------------------------------------*/
///* Private variables ---------------------------------------------------------*/
//
///* Table of different Addresses (devices) */
//const char* RC5_Devices[32] = {
//        "TV1" ,                  /* 0 */
//        "TV2" ,                  /* 1 */
//        "Video Text" ,           /* 2 */
//        "Extension TV",          /* 3 */
//        "LaserVideoPlayer",      /* 4 */
//        "VCR1",                  /* 5 */
//        "VCR2",                  /* 6 */
//        "Reserved",              /* 7 */
//        "Sat1",                  /* 8 */
//        "Extension VCR",         /* 9 */
//        "Sat2",                  /* 10 */
//        "Reserved",              /* 11 */
//        "CD Video",              /* 12 */
//        "Reserved",              /* 13 */
//        "CD Photo",              /* 14 */
//        "Reserved",              /* 15 */
//        "Preampli Audio 1",      /* 16 */
//        "Tuner",                 /* 17 */
//        "Analog Magneto",        /* 18 */
//        "Preampli Audio 2",      /* 19 */
//        "CD",                    /* 20 */
//        "Rack Audio",            /* 21 */
//        "Audio Sat Receiver",    /* 22 */
//        "DDC Magneto",           /* 23 */
//        "Reserved",              /* 24 */
//        "Reserved",              /* 25 */
//        "CDRW",                  /* 26 */
//        "Reserved",              /* 27 */
//        "Reserved",              /* 28 */
//        "Reserved",              /* 29 */
//        "Reserved",              /* 30 */
//        "Reserved"               /* 31 */
//       };
//
///* Table of different commands (TV) */
//const char* RC5_Commands[128] = {
//        "Num0",                                       /* 0 */
//        "Num1",                                       /* 1 */
//        "Num2",                                       /* 2 */
//        "Num3",                                       /* 3 */
//        "Num4",                                       /* 4 */
//        "Num5",                                       /* 5 */
//        "Num6",                                       /* 6 */
//        "Num7",                                       /* 7 */
//        "Num8",                                       /* 8 */
//        "Num9",                                       /* 9 */
//        "TV Digits",                                  /* 10 */
//        "TV Freq",                                    /* 11 */
//        "TV StandBy",                                 /* 12 */
//        "TV Mute On-Off",                             /* 13 */
//        "TV Preference",                              /* 14 */
//        "TV Display",                                 /* 15 */
//        "Volume Up",                                  /* 16 */
//        "Volume Down",                                /* 17 */
//        "Brightness Up",                              /* 18 */
//        "Brightness Down",                            /* 19 */
//        "Color Saturation Up",                        /* 20 */
//        "Color Saturation Down",                      /* 21 */
//        "Bass Up",                                    /* 22 */
//        "Bass Down",                                  /* 23 */
//        "Treble Up",                                  /* 24 */
//        "Treble Down",                                /* 25 */
//        "Balance Right",                              /* 26 */
//        "BalanceLeft",                                /* 27 */
//        "TV Contrast Up",                             /* 28 */
//        "TV Contrast Down",                           /* 29 */
//        "TV Search Up",                               /* 30 */
//        "TV tint-hue Down",                           /* 31 */
//        "TV CH Prog Up",                              /* 32 */
//        "TV CH ProgDown",                             /* 33 */
//        "TV Last viewed program-channel",             /* 34 */
//        "TV Select stereo sound channel-language",    /* 34 */
//        "TV Spacial Stereo",                          /* 36 */
//        "TV Stereo Mono",                             /* 37 */
//        "TV Sleep Timer",                             /* 38 */
//        "TV tint-hue Up",                             /* 39 */
//        "TV RF Switch",                               /* 40 */
//        "TV Store-VOTE",                              /* 41 */
//        "TV Time",                                    /* 42 */
//        "TV Scan Fwd Inc",                            /* 43 */
//        "TV Decrement",                               /* 44 */
//        "Reserved",                                   /* 45 */
//        "TV Secondary control-menu",                  /* 46 */
//        "TV Show Clock",                              /* 47 */
//        "TV Pause",                                   /* 48 */
//        "TV Erase Correct Entry",                     /* 49 */
//        "TV Rewind",                                  /* 50 */
//        "TV Goto",                                    /* 51 */
//        "TV Wind",                                    /* 52 */
//        "TV Play",                                    /* 53 */
//        "TV Stop",                                    /* 54 */
//        "TV Record",                                  /* 55 */
//        "TV External 1",                              /* 56 */
//        "TV External 2",                              /* 57 */
//        "Reserved",                                   /* 58 */
//        "TV Advance",                                 /* 59 */
//        "TV TXT-TV toggle",                           /* 60 */
//        "TV System StandBy",                          /* 61 */
//        "TV Picture Crispener",                       /* 62 */
//        "System Select",                              /* 63 */
//        "Reserved",                                   /* 64 */
//        "Reserved",                                   /* 65 */
//        "Reserved",                                   /* 66 */
//        "Reserved",                                   /* 67 */
//        "Reserved",                                   /* 68 */
//        "Reserved",                                   /* 69 */
//        "TV Speech Music",                            /* 70 */
//        "DIM Local Display",                          /* 71 */
//        "Reserved",                                   /* 72 */
//        "Reserved",                                   /* 73 */
//        "Reserved",                                   /* 74 */
//        "Reserved",                                   /* 75 */
//        "Reserved",                                   /* 76 */
//        "Increase Linear Control Setting",            /* 77 */
//        "Decrease Linear Control Setting",            /* 78 */
//        "TV Sound Scroll",                            /* 79 */
//        "Step Up",                                    /* 80 */
//        "Step Down",                                  /* 81 */
//        "Menu On",                                    /* 82 */
//        "Menu Off",                                   /* 83 */
//        "AV Status",                                  /* 84 */
//        "Step Left",                                  /* 85 */
//        "Step Right",                                 /* 86 */
//        "Acknowledge",                                /* 87 */
//        "PIP On Off",                                 /* 88 */
//        "PIP Shift",                                  /* 89 */
//        "PIP Main Swap",                              /* 90 */
//        "Strobe On Off",                              /* 91 */
//        "Multi Strobe",                               /* 92 */
//        "Main Frozen",                                /* 93 */
//        "3div9 Multi scan",                           /* 94 */
//        "PIPSelect",                                  /* 95 */
//        "MultiPIP",                                   /* 96 */
//        "Picture DNR",                                /* 97 */
//        "Main Stored",                                /* 98 */
//        "PIP Strobe",                                 /* 99 */
//        "Recall Main Stored Picture",                 /* 100 */
//        "PIP Freeze",                                 /* 101 */
//        "PIP Step Up",                                /* 102 */
//        "PIP Step Down",                              /* 103 */
//        "TV PIP Size",                                /* 104 */
//        "TV Picture Scroll",                          /* 105 */
//        "TV Actuate Colored Or Other Special Keys",   /* 106 */
//        "TV Red",                                     /* 107 */
//        "TV Green",                                   /* 108 */
//        "TV Yellow",                                  /* 109 */
//        "TV Cyan",                                    /* 110 */
//        "TV Index White",                             /* 111 */
//        "TV Next",                                    /* 112 */
//        "TV Previous",                                /* 113 */
//        "Reserved",                                   /* 114 */
//        "Reserved",                                   /* 115 */
//        "Reserved",                                   /* 116 */
//        "Reserved",                                   /* 117 */
//        "Sub Mode",                                   /* 118 */
//        "Option Sub Mode",                            /* 119 */
//        "Reserved",                                   /* 120 */
//        "Reserved",                                   /* 121 */
//        "TV Store Open Close",                        /* 122 */
//        "Connect",                                    /* 123 */
//        "Disconnect",                                 /* 124 */
//        "Reserved",                                   /* 125 */
//        "TV Movie Expand",                            /* 126 */
//        "TV Parental Access"                          /* 127 */
//   };
//
//extern StatusYesOrNo RC5_FrameReceived;
//RC5Frame_TypeDef RC5_Frame;
//uint8_t RC5_TogglePrevious = 0;
//StatusYesOrNo FirstTimeIssued = YES;
//
///* Private function prototypes -----------------------------------------------*/
//void RCC_Configuration(void);
//
//
//#ifdef __GNUC__
//  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//     set to 'Yes') calls __io_putchar() */
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */
//
///* Private functions ---------------------------------------------------------*/
//
///**
//  * @brief  Main program
//  * @param  None
//  * @retval None
//  */
//int main(void)
//{
//
//  /* System Clocks Configuration */
//  RCC_Configuration();
//
//  /* Initialize RC5 reception */
//  RC5_Receiver_Init();
//
//  /* Configure LED1 GPIO */
//  //STM_EVAL_LEDInit(LED4);
//  //STM_EVAL_LEDOn(LED4);
//
//
//
//  while (1)
//  {
//     /* If RC5 frame has been received, then decode it */
//	  STM_EVAL_LEDInit(LED);
//	  STM_EVAL_LEDOn(LED4);
//     if (RC5_FrameReceived == YES)
//     {
//         /* Turn On LED1  */
//         STM_EVAL_LEDOn(LED1);
//
//         /* Get the RC5 frame */
//         RC5_Frame = RC5_Decode();
//
//         /* If the first time RC5 frame is received */
//         if(FirstTimeIssued == YES)
//         {
//           /* Get and invert the toggle bit received at the first time to allow
//             to enter the next condition at the first time */
//           RC5_TogglePrevious = ~(RC5_Frame.ToggleBit)&0x01;
//
//           /* Initialize the variable to avoid to enter in this section next time */
//           FirstTimeIssued = NO;
//         }
//
//         if(RC5_Frame.ToggleBit != RC5_TogglePrevious)
//		{
//		  /* Print RC5 Toggle bit state */
//		 // printf("\n\rRC5 Toggle bit: %d \n\r",RC5_Frame.ToggleBit);
//        	 RC5_TogglePrevious = RC5_Frame.ToggleBit;
//		}
//         /* Turn Off LED1  */
//         STM_EVAL_LEDOff(LED1);
//
//
//     }
//  }
//
//
//}
//
//
///**
//  * @brief  Configures the different system clocks.
//  * @param  None
//  * @retval None
//  */
//void RCC_Configuration(void)
//{
//  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,
//     initialize the PLL and update the SystemFrequency variable. */
//  SystemInit();
//
//}
//
//
//
//
///**
//  * @brief  Retargets the C library printf function to the USART.
//  * @param  None
//  * @retval None
//  */
//
//

//
///**
//  * @}
//  */
//
///******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/


#include "stm32f10x.h"
#include "RC5_IR_Emul_Receiver.h"
//#include  "stm3210b_eval.h"

//#include "stm32_eval.h"

#include <stdio.h>

uint8_t i=0;

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}
#endif

//=====
extern StatusYesOrNo RC5_FrameReceived;
RC5Frame_TypeDef RC5_Frame;
uint8_t RC5_TogglePrevious = 0;
StatusYesOrNo FirstTimeIssued = YES;
//=====



void RCC_Configuration(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();

}

void SetupUSART();

int main(void)
{
  /* System Clocks Configuration */
  RCC_Configuration();

  /* Initialize RC5 reception */
  RC5_Receiver_Init();

//  SetupUSART();


  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;  // Enable PORTC Periph clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // Enable TIM3 Periph clock
  // Clear PC8 and PC9 control register bits
  GPIOC->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8 |
          GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
  // Configure PC8 and PC9 as Push Pull output at max 10Mhz
  GPIOC->CRH |= GPIO_CRH_MODE8_0 | GPIO_CRH_MODE9_0;
  TIM3->PSC = SystemCoreClock / 1000 - 1; // 1000 tick/sec
  TIM3->ARR = 1000;  // 1 Interrupt/sec (1000/100)
  TIM3->DIER |= TIM_DIER_UIE; // Enable TIM3 interrupt
  TIM3->CR1 |= TIM_CR1_CEN;   // Start count
  NVIC_EnableIRQ(TIM3_IRQn);  // Enable IRQ

  while(1) // Infinity loop
  {

	   if (RC5_FrameReceived == YES)
	   {
		   /* Turn On LED1  */
		//   STM_EVAL_LEDOn(LED1);

		   /* Get the RC5 frame */
		   RC5_Frame = RC5_Decode();

//		   USART_SendData(USART1,RC5_Frame.Command);
	   }
  }
}


/***************************************************************************//**
 * @brief Init USART1
 ******************************************************************************/
void SetupUSART()
{

      GPIO_InitTypeDef  GPIO_InitStructure;
      USART_InitTypeDef USART_InitStructure;

      /* Enable GPIOA clock                                                   */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

      /* Configure USART1 Rx (PA10) as input floating                         */
      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      /* Configure USART1 Tx (PA9) as alternate function push-pull            */
      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
      /* USART1 configured as follow:
            - BaudRate = 9600 baud
            - Word Length = 8 Bits
            - One Stop Bit
            - No parity
            - Hardware flow control disabled (RTS and CTS signals)
            - Receive and transmit enabled
            - USART Clock disabled
            - USART CPOL: Clock is active low
            - USART CPHA: Data is captured on the middle
            - USART LastBit: The clock pulse of the last data bit is not output to
                             the SCLK pin
      */
      USART_InitStructure.USART_BaudRate            = 9600;
      USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits            = USART_StopBits_1;
      USART_InitStructure.USART_Parity              = USART_Parity_No ;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
      USART_Init(USART1, &USART_InitStructure);
      USART_Cmd(USART1, ENABLE);
}
void TIM3_IRQHandler(void)
{

//	/* Sample each bit of RC5 frame */
//	RC5_Sample_Data();

  TIM3->SR &= ~TIM_SR_UIF; //Clean UIF Flag
  if (1 == (i++ & 0x1)) {
    GPIOC->BSRR = GPIO_BSRR_BS8;   // Set PC8 bit
    GPIOC->BSRR = GPIO_BSRR_BR9;   // Reset PC9 bit
  } else {
    GPIOC->BSRR = GPIO_BSRR_BS9;   // Set PC9 bit
    GPIOC->BSRR = GPIO_BSRR_BR8;   // Reset PC8 bit
  }
}

