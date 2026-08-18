#ifndef INC_HC_SR04_H_
#define INC_HC_SR04_H_

#include "main.h"
#include "tim.h"

void HCSR04_Init(void);
uint16_t HCSR04_Get_Distance(void);

#endif /* INC_HC_SR04_H_ */
