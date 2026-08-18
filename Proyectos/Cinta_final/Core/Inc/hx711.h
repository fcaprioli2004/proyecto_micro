/*
 * hx711.h
 *
 *  Created on: 18 jul 2026
 *      Author: Fausto Caprioli
 */

#ifndef HX711_H
#define HX711_H

#include "main.h"
#include <stdint.h>

extern int32_t tare;
extern float calibrationFactor;

void HX711_Init(void);
int32_t getHX711(void);
int32_t HX711_ReadAverage(uint16_t samples);
HAL_StatusTypeDef HX711_Tare(uint16_t samples);
HAL_StatusTypeDef HX711_Calibrate(float referenceWeightMg, uint16_t samples);
int32_t HX711_Weigh(uint16_t samples);
HAL_StatusTypeDef HX711_WeighStatus(uint16_t samples,int32_t *weightMg);
HAL_StatusTypeDef HX711_WeighNonBlocking(int32_t *weightMg);

#endif
