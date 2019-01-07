#ifndef _OV7725_H_
#define _OV7725_H_

#define OV7725_IRQHandler EXTI0_IRQHandler
#define OV7725_EXTI_Line  EXTI_Line0

#include "stm32f4xx.h"

#include <stdbool.h>

#define OV7725_IMAGE_SIZE 240

bool OV7725_Init(void);

extern __inline void     FIFO_PointReset(void);
extern __inline void     FIFO_PointMove (void);
extern __inline void     FIFO_Pause     (void);
extern __inline void     FIFO_Prepare   (void);
extern __inline uint16_t FIFO_ReadPixl  (void);

#endif
