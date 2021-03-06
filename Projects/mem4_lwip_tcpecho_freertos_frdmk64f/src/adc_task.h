/*
 * adc_task.h
 *
 *  Created on: Feb 23, 2016
 *      Author: mortenopprudjakobsen
 */

#ifndef SOURCE_ADC_TASK_H_
#define SOURCE_ADC_TASK_H_

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#define NO_SAMPLES	4096 //1024
//unsigned int sample_index = 0;
//unsigned int sample_buff_in_use = 0;

extern  uint8_t sample_buff_in_use;
extern  uint8_t adcSamples0[2*NO_SAMPLES];
extern  uint8_t adcSamples1[2*NO_SAMPLES];
extern  uint8_t adcSamples2[2*NO_SAMPLES];


void startSampling(void);
void adc_init(void);
void adc_task(void *pvParameters);
void init_trigger_source(uint32_t adcInstance);

/* RTOS resources */
static volatile SemaphoreHandle_t xSemaphore = NULL;
static volatile QueueHandle_t sampleQhdl;




#endif /* SOURCE_ADC_TASK_H_ */
