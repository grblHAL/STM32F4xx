/*
 * f4mpv4_hosoi.h
 *
 *  Created on: Dec 23, 2021
 *      Author: tai
 */

#ifndef F4MPV4_HOSOI_H_
#define F4MPV4_HOSOI_H_

#define PU1_Pin GPIO_PIN_2
#define PU1_GPIO_Port GPIOA
#define DR1_Pin GPIO_PIN_3
#define DR1_GPIO_Port GPIOA
#define LedG_Pin GPIO_PIN_6
#define LedG_GPIO_Port GPIOC
#define LedB_Pin GPIO_PIN_7
#define LedB_GPIO_Port GPIOC
#define LedR_Pin GPIO_PIN_8
#define LedR_GPIO_Port GPIOC

#define led1		  GPIOC,GPIO_PIN_6	 //GR
#define led2		  GPIOC,GPIO_PIN_7	 //BL
#define led3		  GPIOC,GPIO_PIN_8	 //RD
#define led1on		HAL_GPIO_WritePin(led1,GPIO_PIN_SET)
#define led2on		HAL_GPIO_WritePin(led2,GPIO_PIN_SET)
#define led3on		HAL_GPIO_WritePin(led3,GPIO_PIN_SET)
#define led1off		HAL_GPIO_WritePin(led1,GPIO_PIN_RESET)
#define led2off		HAL_GPIO_WritePin(led2,GPIO_PIN_RESET)
#define led3off		HAL_GPIO_WritePin(led3,GPIO_PIN_RESET)
#define led1toggle  HAL_GPIO_TogglePin(led1);
#define led2toggle  HAL_GPIO_TogglePin(led2);
#define led3toggle  HAL_GPIO_TogglePin(led3);

#define  X0		GPIOE,GPIO_PIN_0
#define  X1		GPIOE,GPIO_PIN_1
#define  X2		GPIOE,GPIO_PIN_2
#define  X3		GPIOE,GPIO_PIN_3
#define  X4		GPIOE,GPIO_PIN_4
#define  X5		GPIOE,GPIO_PIN_5
#define  X6		GPIOE,GPIO_PIN_6
#define  X7		GPIOE,GPIO_PIN_7
#define  X8		GPIOE,GPIO_PIN_8
#define  X9		GPIOE,GPIO_PIN_9
#define  X10	GPIOE,GPIO_PIN_10
#define  X11	GPIOE,GPIO_PIN_11

#define X0R       HAL_GPIO_ReadPin(X0)
#define X1R       HAL_GPIO_ReadPin(X1)
#define X2R       HAL_GPIO_ReadPin(X2)
#define X3R       HAL_GPIO_ReadPin(X3)
#define X4R       HAL_GPIO_ReadPin(X4)
#define X5R       HAL_GPIO_ReadPin(X5)
#define X6R       HAL_GPIO_ReadPin(X6)
#define X7R       HAL_GPIO_ReadPin(X7)
#define X8R       HAL_GPIO_ReadPin(X8)
#define X9R       HAL_GPIO_ReadPin(X9)
#define X10R      HAL_GPIO_ReadPin(X10)
#define X11R      HAL_GPIO_ReadPin(X11)

#define  X0set	HAL_GPIO_WritePin(X0, GPIO_PIN_SET);
#define  X1set	HAL_GPIO_WritePin(X1, GPIO_PIN_SET);
#define  X2set	HAL_GPIO_WritePin(X2, GPIO_PIN_SET);
#define  X3set	HAL_GPIO_WritePin(X3, GPIO_PIN_SET);
#define  X4set	HAL_GPIO_WritePin(X4, GPIO_PIN_SET);
#define  X5set	HAL_GPIO_WritePin(X5, GPIO_PIN_SET);
#define  X6set	HAL_GPIO_WritePin(X6, GPIO_PIN_SET);
#define  X7set	HAL_GPIO_WritePin(X7, GPIO_PIN_SET);
#define  X8set	HAL_GPIO_WritePin(X8, GPIO_PIN_SET);
#define  X9set	HAL_GPIO_WritePin(X9, GPIO_PIN_SET);
#define  X10set	HAL_GPIO_WritePin(X10, GPIO_PIN_SET);
#define  X11set	HAL_GPIO_WritePin(X11, GPIO_PIN_SET);

#define  X0clr 	HAL_GPIO_WritePin(X0, GPIO_PIN_RESET);
#define  X1clr 	HAL_GPIO_WritePin(X1, GPIO_PIN_RESET);
#define  X2clr 	HAL_GPIO_WritePin(X2, GPIO_PIN_RESET);
#define  X3clr 	HAL_GPIO_WritePin(X3, GPIO_PIN_RESET);
#define  X4clr 	HAL_GPIO_WritePin(X4, GPIO_PIN_RESET);
#define  X5clr 	HAL_GPIO_WritePin(X5, GPIO_PIN_RESET);
#define  X6clr 	HAL_GPIO_WritePin(X6, GPIO_PIN_RESET);
#define  X7clr 	HAL_GPIO_WritePin(X7, GPIO_PIN_RESET);
#define  X8clr 	HAL_GPIO_WritePin(X8, GPIO_PIN_RESET);
#define  X9clr 	HAL_GPIO_WritePin(X9, GPIO_PIN_RESET);
#define  X10clr HAL_GPIO_WritePin(X10, GPIO_PIN_RESET);
#define  X11clr HAL_GPIO_WritePin(X11, GPIO_PIN_RESET);

#define  X0tog 	HAL_GPIO_TogglePin(X0);
#define  X1tog 	HAL_GPIO_TogglePin(X1);
#define  X2tog 	HAL_GPIO_TogglePin(X2);
#define  X3tog 	HAL_GPIO_TogglePin(X3);
#define  X4tog 	HAL_GPIO_TogglePin(X4);
#define  X5tog 	HAL_GPIO_TogglePin(X5);
#define  X6tog 	HAL_GPIO_TogglePin(X6);
#define  X7tog 	HAL_GPIO_TogglePin(X7);
#define  X8tog 	HAL_GPIO_TogglePin(X8);
#define  X9tog 	HAL_GPIO_TogglePin(X9);
#define  X10tog HAL_GPIO_TogglePin(X10);
#define  X11tog HAL_GPIO_TogglePin(X11);

#endif /* F4MPV4_HOSOI_H_ */
