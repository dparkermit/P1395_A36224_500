#ifndef __A36224_H
#define __A36224_H

#include <p30f6014a.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>

#include "ETM_CAN_PUBLIC.h"
#include "ETM_ANALOG.h"




/*
  Hardware Module Resource Usage

  CAN2   - Used/Configured by ETM CAN 
  Timer2 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer3 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI1   - Used/Configured by MCP4822 Modules

  Timer5 - Used for 10msTicToc

  ADC Module - See Below For Specifics

*/



// ------------------------ CONFIGURE ADC MODULE ------------------- //

// ----------------- ANALOG INPUT PINS ---------------- //
/* 
   AN2  - Flow 0 - UNUSED
   AN3  - Flow 1 - UNUSED
   AN4  - Flow 2 - UNUSED
   AN5  - Flow 3 - UNUSED
   AN6  - Flow 4 - UNUSED
   AN7  - Flow 5 - UNUSED
   
   AN8  - Analog 0 - Electromagnet Current
   AN9  - Analog 1 - Heater Current
   AN10 - Analog 2 - Electromagnet Voltage
   AN11 - Analog 3 - Heater Voltage

   
*/

/*
  This sets up the ADC to work as following
  AUTO Sampeling
  VDD / GND as reference
  
  With 10MHz System Clock, ADC Clock is 450ns, Sample Time is 6 ADC Clock so total sample time is 9uS
  Conversion rate of 111KHz (27.8 Khz per Channel), 277 Samples per 10mS interrupt
  
  8 Samples per Interrupt, use alternating buffers
  Ignore the flow Inputs, only scan the voltage/current monitors -  AN8,AN9,AN10,AN11

*/

#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_AVDD_AVSS & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_8 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING          (ADC_SAMPLE_TIME_4 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_9Tcy2)
#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN3 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN3 & ADC_CH0_NEG_SAMPLEB_VREFN)
#define ADPCFG_SETTING          (ENABLE_AN2_ANA & ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN6_ANA & ENABLE_AN7_ANA & ENABLE_AN8_ANA & ENABLE_AN9_ANA & ENABLE_AN10_ANA & ENABLE_AN11_ANA)
#define ADCSSL_SETTING          (SKIP_SCAN_AN2 & SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN5 & SKIP_SCAN_AN6 & SKIP_SCAN_AN7)





/* 
   TMR5 Configuration
   Timer5 - Used for 10msTicToc
   Period should be set to 10mS
   With 10Mhz Clock, x8 multiplier will yield max period of 17.7mS, 2.71uS per tick
*/

#define T5CON_VALUE                    (T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_8 & T5_SOURCE_INT)
#define PR5_PERIOD_US                  10000   // 10mS
#define PR5_VALUE_10_MILLISECONDS      (FCY_CLK_MHZ*PR5_PERIOD_US/8)



// -------------------- A36224_500 STATUS BIT CONFIGURATION ------------------------ //
#define STATUS_SOFTWARE_DISABLE                ETM_CAN_STATUS_WORD_0_USER_DEFINED_8
#define STATUS_READBACK_ELECTROMAGNET_STATUS   ETM_CAN_STATUS_WORD_0_USER_DEFINED_9
#define STATUS_READBACK_HEATER_STATUS          ETM_CAN_STATUS_WORD_0_USER_DEFINED_10
#define STATUS_READBACK_HEATER_OV_STATUS       ETM_CAN_STATUS_WORD_0_USER_DEFINED_11
#define STATUS_READBACK_CROWBAR_STATUS         ETM_CAN_STATUS_WORD_0_USER_DEFINED_12
#define STATUS_READBACK_TEMPERATURE_STATUS     ETM_CAN_STATUS_WORD_0_USER_DEFINED_13

#define STAUS_INHIBIT_MASK                     (ETM_CAN_STATUS_WORD_0_BOARD_RECEIVED_INITIAL_CONFIG | STATUS_SOFTWARE_DISABLE) // DPAREKR ADD ALL THE STATUS



// -------------------- A36224_500 FAULTS/WARNINGS CONFIGURATION-------------------- //
#define FAULT_HEATER_OVER_CUR_ABSOLUTE         ETM_CAN_STATUS_WORD_1_FAULT_USER_DEFINED_1
#define FAULT_HEATER_UNDER_CUR_ABSOLUTE        ETM_CAN_STATUS_WORD_1_FAULT_USER_DEFINED_2
#define FAULT_HEATER_OVER_CUR_RELATIVE         ETM_CAN_STATUS_WORD_1_FAULT_USER_DEFINED_3
#define FAULT_HEATER_UNDER_CUR_RELATIVE        ETM_CAN_STATUS_WORD_1_FAULT_USER_DEFINED_4
#define FAULT_HEATER_OVER_VOL_ABSOULTE         ETM_CAN_STATUS_WORD_1_FAULT_USER_DEFINED_5
#define FAULT_HEATER_UNDER_VOL_ABSOLUTE        ETM_CAN_STATUS_WORD_1_FAULT_USER_DEFINED_6

#define FAULT_MAGNET_OVER_CUR_ABSOLUTE         ETM_CAN_STATUS_WORD_1_FAULT_USER_DEFINED_7
#define FAULT_MAGNET_UNDER_CUR_ABSOLUTE        ETM_CAN_STATUS_WORD_1_FAULT_USER_DEFINED_8
#define FAULT_MAGNET_OVER_CUR_RELATIVE         ETM_CAN_STATUS_WORD_1_FAULT_USER_DEFINED_9
#define FAULT_MAGNET_UNDER_CUR_RELATIVE        ETM_CAN_STATUS_WORD_1_FAULT_USER_DEFINED_10
#define FAULT_MAGNET_OVER_VOL_ABSOULTE         ETM_CAN_STATUS_WORD_1_FAULT_USER_DEFINED_11
#define FAULT_MAGNET_UNDER_VOL_ABSOLUTE        ETM_CAN_STATUS_WORD_1_FAULT_USER_DEFINED_12

#define FAULT_MASK                             (ETM_CAN_STATUS_WORD_1_FAULT_CAN_BUS | FAULT_HEATER_OVER_CUR_ABSOLUTE) // DPARKER ADD ALL THE FAULTS



// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set

// ----------------- DIGITAL INPUT PINS --------------- //
/*

  RA12 - Digital in 4
  RA13 - Digital in 4
  RA14 - Digital IN 5
  RA15 - Digital IN 5

  RB12 - Digital IN 0
  RB13 - Digital IN 1
  RB14 - Digital IN 2
  RB15 - DIgital IN 3

  RF3  - Optical IN - This is Blue Wired

  --------- Pins that are overidden by a hardware module and should be left as inputs during port configuration ----
  RB0 - PROGRAM
  RB1 - PROGRAM

  RB2  - Analog Input
  RB3  - Analog Input
  RB4  - Analog Input
  RB5  - Analog Input
  RB6  - Analog Input
  RB7  - Analog Input
  RB8  - Analog Input
  RB9  - Analog Input
  RB10 - Analog Input
  RB11 - Analog Input

  RG0 - CAN 2
  RG1 - CAN 2
  RG6 - SPI 2
  RG8 - SPI 2

  -------- Pins that are configured by other software modules and should be left as inputs during port configuration -----------
  RD14 (DAC2 SELECT)
  RD15 (DAC1 SELECT)
  RF5  (DAC LDAC)


*/

#define A36444_TRISA_VALUE 0b1111000000000000
#define A36444_TRISB_VALUE 0b1111111111111111
#define A36444_TRISC_VALUE 0b0000000000000000
#define A36444_TRISD_VALUE 0b1100000000000000
#define A36444_TRISF_VALUE 0b0000000000001000
#define A36444_TRISG_VALUE 0b0000000101100011


//   ------------------  Digital Output Pins ---------------
/*

  RC1  - LED
  RC2  - LED
  RC3  - LED
  RC4  - LED

  RC14 - Optical Out

  RD0  - Digital Out 0
  RD1  - Digital Out 1
  RD2  - Digital Out 2
  RD3  - Digital Out 3
  RD4  - Refresh


  RG12 - LED Watchdog
  RG13 - LED COM
  RG14 - LED Power
  RG15 - LED Flow
*/


// -------- Digital Input Pins ----------//
#define PIN_D_IN_0_ELECTROMAGENT_STATUS            _RB12        
#define PIN_D_IN_1_HEATER_STATUS                   _RB13
#define PIN_D_IN_2_UNUSED                          _RB14
#define PIN_D_IN_3_HEATER_OVER_VOLT_STATUS         _RB15
#define PIN_D_IN_4_CROWBAR_STATUS                  _RA12
#define PIN_D_IN_5_TEMPERATURE_STATUS              _RA14   

#define PIN_OPTICAL_IN_UNUSED                      _RF3

// ------- Digital Output Pins ---------//

#define PIN_D_OUT_0                                _LATD0
#define PIN_D_OUT_1                                _LATD1
#define PIN_D_OUT_REFRESH                          _LATD4

#define PIN_D_OUT_2_ELECTROMAGENT_ENABLE           _LATD2
#define PIN_D_OUT_3_HEATER_ENABLE                  _LATD3

#define PIN_LED_WATCHDOG                           _LATG12
#define PIN_LED_COM                                _LATG13
#define PIN_LED_POWER                              _LATG14
#define PIN_LED_FLOW                               _LATG15
#define PIN_LED_I2_A                               _LATC1
#define PIN_LED_I2_B                               _LATC2
#define PIN_LED_I2_C                               _LATC3
#define PIN_LED_I2_D                               _LATC4


typedef struct {
  // all currents are scaled to 1mA per lsb
  // all voltages are scaled to 1mV per lsb

  AnalogInput analog_input_heater_voltage;
  AnalogInput analog_input_heater_current;

  AnalogInput analog_input_electromagnet_voltage;
  AnalogInput analog_input_electromagnet_current;

  AnalogOutput analog_output_heater_current;
  AnalogOutput analog_output_electromagnet_current;

  unsigned int  accumulator_counter;

  unsigned int  adc_ignore_current_sample;

} HeaterMagnetControlData;

extern HeaterMagnetControlData global_data_A36224_500;



#endif
