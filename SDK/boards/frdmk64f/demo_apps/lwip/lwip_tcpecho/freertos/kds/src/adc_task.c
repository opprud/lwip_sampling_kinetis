/*
 * adc_task.c
 *
 *  Created on: Feb 23, 2016
 *      Author: mortenopprudjakobsen
 */

#include <string.h>

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_adc16.h"
#include "adc_task.h"

#include "fsl_pit.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U

#define DEMO_ADC16_IRQn ADC0_IRQn
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

/*******************************************************************************
 * module variables
 ******************************************************************************/
  uint8_t adcSamples0[2*NO_SAMPLES];
  uint8_t adcSamples1[2*NO_SAMPLES];
  uint8_t adcSamples2[2*NO_SAMPLES];


static  adc16_channel_config_t adc16ChannelConfigStruct;

//unsigned short adcSamples[2][NO_SAMPLES];
//unsigned int sample_index = 0;
//unsigned int sample_buff_in_use = 0;

/* periodic interrupt timer */
void PIT0_IRQHandler(void)
{
	short dummy;
	GPIO_TogglePinsOutput(GPIOE, 1U << 24);

	/* read previous conversion */
	/* Read conversion result to clear the conversion completed flag. */
	//dummy = ADC16_GetChannelConversionValue(ADC0, 0);

#if 1
//	adcSamples[sample_buff_in_use][sample_index] = (unsigned short) ADC16_GetChannelConversionValue(ADC0, 0);
	dummy = ADC16_GetChannelConversionValue(ADC0, 0);

	if(sample_buff_in_use==0)
	{
		adcSamples0[sample_index++] = (uint8_t)(dummy & 0x00ff); //rand();//(unsigned short) ADC16_GetChannelConversionValue(ADC0, 0);
		adcSamples0[sample_index++] = (uint8_t)((dummy>>8) & 0x00ff); //rand();//(unsigned short) ADC16_GetChannelConversionValue(ADC0, 0);
	}
	else if(sample_buff_in_use==1)
	{
		adcSamples1[sample_index++] = (uint8_t)(dummy & 0x00ff); //rand();//(unsigned short) ADC16_GetChannelConversionValue(ADC0, 0);
		adcSamples1[sample_index++] = (uint8_t)((dummy>>8) & 0x00ff); //rand();//(unsigned short) ADC16_GetChannelConversionValue(ADC0, 0);
	}
	else
	{
		adcSamples2[sample_index++] = (uint8_t)(dummy & 0x00ff); //rand();//(unsigned short) ADC16_GetChannelConversionValue(ADC0, 0);
		adcSamples2[sample_index++] = (uint8_t)((dummy>>8) & 0x00ff); //rand();//(unsigned short) ADC16_GetChannelConversionValue(ADC0, 0);
	}

	/* buffer is full, notify and update */
	if (sample_index >= 2*NO_SAMPLES)
	{
		sample_index = 0;
		if (sample_buff_in_use == 0)
			sample_buff_in_use = 1;
		else if (sample_buff_in_use == 1)
			sample_buff_in_use = 2;
		else
			sample_buff_in_use = 0;
	}
#endif
	/* Clear interrupt flag.*/
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, PIT_TFLG_TIF_MASK);
	//pitIsrFlag = true;
	adc16ChannelConfigStruct.channelNumber = 12;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false; /* Enable the interrupt. */
	adc16ChannelConfigStruct.enableDifferentialConversion = false;

	/*start ADC again */
	ADC16_SetChannelConfig(ADC0, 0,&adc16ChannelConfigStruct);
}



void startSampling(void)
{
	/* !IMPORTANT !
	 * set priority to allow OS function from within interrupt*/
	//NVIC_SetPriority(PIT0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	EnableIRQ(PIT0_IRQn);
	//NVIC_SetPriority(ADC0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	//EnableIRQ(ADC0_IRQn);

	/* Start channel 0 */
	PRINTF("\r\nStarting channel No.0 ...");
	PIT_StartTimer(PIT, kPIT_Chnl_0);
}

void stopSampling(void)
{
	PIT_StopTimer(PIT, kPIT_Chnl_0);
}

/*!
 * @Brief enable the trigger source of PIT0, chn0
 */
void init_trigger_source(uint32_t adcInstance)
{
	uint32_t freqUs;

	freqUs = 1000000U / NO_SAMPLES;
	/* Structure of initialize PIT */
	pit_config_t pitConfig;
	/*pitConfig.enableRunInDebug = false; */
	PIT_GetDefaultConfig(&pitConfig);
	/* Init pit module */
	PIT_Init(PIT, &pitConfig);
	/* Set timer period for channel 0 */
//	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(freqUs, PIT_SOURCE_CLOCK));
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(100, PIT_SOURCE_CLOCK));
	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
}

/*!
 * @brief Task responsible for initializing ADC
 */
void adc_init(void)
{
	adc16_config_t adc16ConfigStruct;

	/*
	 * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
	 * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
	 * adc16ConfigStruct.enableAsynchronousClock = true;
	 * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
	 * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
	 * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
	 * adc16ConfigStruct.enableHighSpeed = false;
	 * adc16ConfigStruct.enableLowPower = false;
	 * adc16ConfigStruct.enableContinuousConversion = false;
	 */
	/* run continiously*/
	//adc16ConfigStruct.enableContinuousConversion = true;
	ADC16_GetDefaultConfig(&adc16ConfigStruct);
	ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
	ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false);

	if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
	{
		PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
	}

	adc16ChannelConfigStruct.channelNumber = 12;
//	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
	adc16ChannelConfigStruct.enableDifferentialConversion = false;

	/* run first conversion */
	ADC16_SetChannelConfig(ADC0, 0,&adc16ChannelConfigStruct);


}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
void adc_task(void *pvParameters)
{

	/* setup ADC*/
	adc_init();
	/* and trigger timer */
	init_trigger_source(1);

	vTaskDelay(400);
	startSampling();

	for (;;)
	{

		vTaskDelay(200);
		LED_BLUE_TOGGLE();
	}

}

