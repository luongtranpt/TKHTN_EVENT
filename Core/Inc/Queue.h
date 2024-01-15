/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Queue_H
#define __Queue_H


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "stdlib.h"

/* Private includes ----------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/
#define Queue_Size 10
/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum
{
 None                 = 0,
 IRQ_Event_Timer2     = 1,
 IRQ_Event_Fre        = 2,
 Task_End_Event_Humd  = 3,
 Task_End_Event_Temp  = 4,
 Task_End_LCD         = 5,
 IRQ_Event_UART_TransT= 6,
 IRQ_Event_UART_TransH= 7,
 IRQ_Event_UART_Recv  = 8,
 Task_UART_END        = 9,
 IRQ_Event_ADC        = 10,
 IRQ_Event_Timer3     = 11
}EventState;

typedef enum
{
 PushQueue_Done = 0,
 PopQueue_Done  = 2,
 PopQueue_Fail  = 3,
 Queue_Empty    = 4,
 Queue_Full     = 5
}QueueState;

typedef struct _queue
{ 
  //QueueState Result;
  EventState State;
  void(*Func)(void);
  uint16_t data;
  uint8_t array[6];
}queue_t;

/* Exported functions prototypes ---------------------------------------------*/
QueueState Push_Queue( queue_t*,EventState ,void(*)(void),uint16_t,uint8_t *);
QueueState Pop_Queue ( queue_t*);
uint8_t Sizeof_Queue_Current ( queue_t*);

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#endif

