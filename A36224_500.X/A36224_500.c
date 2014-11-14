#include "A36224_500.h"
#include "MCP4822.h"

_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer 
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);



void DisableHeaterMagnetOutputs(void);
void EnableHeaterMagnetOutputs(void);




void InitializeA36224(void);
void DoA36225_500(void);
void DoStateMachine(void);

MCP4822 U42_MCP4822;
MCP4822 U44_MCP4822;

HeaterMagnetControlData global_data_A36224_500;

unsigned int led_divider;


unsigned int control_state;

#define STATE_STARTUP                0x10
#define STATE_WAITING_FOR_CONFIG     0x20
#define STATE_STANDBY                0x30
#define STATE_OPERATE                0x40
#define STATE_FAULT                  0x50

int main(void) {
  control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}





void DoStateMachine(void) {

  switch (control_state) {
    
  case STATE_STARTUP:
    InitializeA36224();
    control_state = STATE_WAITING_FOR_CONFIG;
    break;
    
  case STATE_WAITING_FOR_CONFIG:
    ETMCanSetBit(&etm_can_status_register.status_word_0, STATUS_BIT_BOARD_WAITING_INITIAL_CONFIG);
    ETMCanSetBit(&etm_can_status_register.status_word_0, STATUS_BIT_SOFTWARE_DISABLE);
    DisableHeaterMagnetOutputs();
    while (control_state == STATE_WAITING_FOR_CONFIG) {
      DoA36225_500();
      ETMCanDoCan();
      
      if (!ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_BOARD_WAITING_INITIAL_CONFIG)) {
	control_state = STATE_STANDBY;
      }
      
      if (ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_SUM_FAULT)) {
	control_state = STATE_FAULT;
      }
    }
    break;

  case STATE_STANDBY:
    DisableHeaterMagnetOutputs();
    while (control_state == STATE_STANDBY) {
      DoA36225_500();
      ETMCanDoCan();
      
      if (!ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_SOFTWARE_DISABLE)) {
	control_state = STATE_OPERATE;
      }
      
      if (ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_SUM_FAULT)) {
	control_state = STATE_FAULT;
      }
    }
    break;

  case STATE_OPERATE:
    EnableHeaterMagnetOutputs();
    while (control_state == STATE_OPERATE) {
      DoA36225_500();
      ETMCanDoCan();

      if (ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_SOFTWARE_DISABLE)) {
	control_state = STATE_STANDBY;
      }
      
      if (ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_SUM_FAULT)) {
	control_state = STATE_FAULT;
      }
    }
    break;


  case STATE_FAULT:
    DisableHeaterMagnetOutputs();
    while (control_state == STATE_FAULT) {
      DoA36225_500();
      ETMCanDoCan();
      if (!ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_SUM_FAULT)) {
	// The faults have been cleared
	control_state = STATE_WAITING_FOR_CONFIG;
      }
    }
    break;
    
    
  default:
    control_state = STATE_FAULT;
    break;

    
  }
  

  
}


void DisableHeaterMagnetOutputs(void) {
  global_data_A36224_500.analog_output_heater_current.enabled = 0;
  global_data_A36224_500.analog_output_electromagnet_current.enabled = 0;
  
  PIN_D_OUT_2_ELECTROMAGNET_ENABLE = !OLL_ENABLE_SUPPLY;
  PIN_D_OUT_3_HEATER_ENABLE = !OLL_ENABLE_SUPPLY;

  PIN_D_OUT_1_OUTPUT_RELAY = !OLL_CLOSE_RELAY;
}


void EnableHeaterMagnetOutputs(void) {
  global_data_A36224_500.analog_output_heater_current.enabled = 1;
  global_data_A36224_500.analog_output_electromagnet_current.enabled = 1;
  
  PIN_D_OUT_2_ELECTROMAGNET_ENABLE = OLL_ENABLE_SUPPLY;
  PIN_D_OUT_3_HEATER_ENABLE = OLL_ENABLE_SUPPLY;

  PIN_D_OUT_1_OUTPUT_RELAY = OLL_CLOSE_RELAY;
}



void DoA36225_500(void) {

  // Check the status of these pins every time through the loop
  if (PIN_D_IN_4_HEATER_OVER_VOLT_STATUS == ILL_HEATER_OV) {
    ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_HW_HEATER_OVER_VOLTAGE);
  }
  
  // Check the status of these pins every time through the loop
  if (PIN_D_IN_5_TEMPERATURE_STATUS == ILL_TEMP_SWITCH_FAULT) {
    ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_HW_TEMPERATURE_SWITCH);
  }

  if (_T5IF) {
    // 10ms Timer has expired so this code will executre once every 10ms
    _T5IF = 0;
    
    // Flash the operate LED
    led_divider++;
    if (led_divider >= 50) {
      led_divider = 0;
      if (PIN_LED_POWER) {
	PIN_LED_POWER = 0;
      } else {
	PIN_LED_POWER = 1;
      }
    }

    // Set the fault LED
    if (etm_can_status_register.status_word_0 & 0x0003) {
      // The board is faulted or inhibiting the system
      PIN_LED_I2_C = 0;
    } else {
      PIN_LED_I2_C = 1;
    }

    // Update the digital input status pins
    if (PIN_D_IN_0_ELECTROMAGENT_STATUS == ILL_POWER_SUPPLY_DISABLED) {
      ETMCanSetBit(&etm_can_status_register.status_word_0, STATUS_BIT_READBACK_ELECTROMAGNET_STATUS);
    } else {
      ETMCanClearBit(&etm_can_status_register.status_word_0, STATUS_BIT_READBACK_ELECTROMAGNET_STATUS);
    }
    
    if (PIN_D_IN_1_HEATER_STATUS == ILL_POWER_SUPPLY_DISABLED) {
      ETMCanSetBit(&etm_can_status_register.status_word_0, STATUS_BIT_READBACK_HEATER_STATUS);
    } else {
      ETMCanClearBit(&etm_can_status_register.status_word_0, STATUS_BIT_READBACK_HEATER_STATUS);
    }
    
    if (PIN_D_IN_3_RELAY_STATUS == ILL_RELAY_OPEN) {
      ETMCanSetBit(&etm_can_status_register.status_word_0, STATUS_BIT_READBACK_RELAY_STATUS);
    } else {
      ETMCanClearBit(&etm_can_status_register.status_word_0, STATUS_BIT_READBACK_RELAY_STATUS);
    }
    
    // Flash the Refresh
    if (PIN_D_OUT_REFRESH) {
      PIN_D_OUT_REFRESH = 0;
    } else {
      PIN_D_OUT_REFRESH = 1;
    }
    
    // Do Math on ADC inputs
    // Scale the ADC readings to engineering units
    ETMAnalogScaleCalibrateADCReading(&global_data_A36224_500.analog_input_electromagnet_current);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36224_500.analog_input_electromagnet_voltage);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36224_500.analog_input_heater_current);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36224_500.analog_input_heater_voltage);
    
#define NOMINAL_ELECTROMAGNET_RESISTANCE            1.00
#define NOMINAL_HEATER_RESISTANCE                   1.00    

    // Check for analog Faults
    global_data_A36224_500.analog_input_electromagnet_current.target_value = global_data_A36224_500.analog_output_electromagnet_current.set_point;
    global_data_A36224_500.analog_input_electromagnet_voltage.target_value = ETMScaleFactor16(global_data_A36224_500.analog_output_electromagnet_current.set_point,MACRO_DEC_TO_SCALE_FACTOR_16(NOMINAL_ELECTROMAGNET_RESISTANCE),0);
    global_data_A36224_500.analog_input_heater_current.target_value = global_data_A36224_500.analog_output_heater_current.set_point;
    global_data_A36224_500.analog_input_heater_voltage.target_value = ETMScaleFactor16(global_data_A36224_500.analog_output_heater_current.set_point,MACRO_DEC_TO_SCALE_FACTOR_16(NOMINAL_HEATER_RESISTANCE),0);


    if (ETMAnalogCheckOverAbsolute(&global_data_A36224_500.analog_input_heater_current)) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_HEATER_OVER_CUR_ABSOLUTE);
    }
    if (ETMAnalogCheckUnderAbsolute(&global_data_A36224_500.analog_input_heater_current)) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_HEATER_UNDER_CUR_ABSOLUTE);
    }
    if (ETMAnalogCheckOverRelative(&global_data_A36224_500.analog_input_heater_current)) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_HEATER_OVER_CUR_RELATIVE);
    }
    if (ETMAnalogCheckUnderRelative(&global_data_A36224_500.analog_input_heater_current)) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_HEATER_UNDER_CUR_RELATIVE);
    }
    if (ETMAnalogCheckOverAbsolute(&global_data_A36224_500.analog_input_heater_voltage)) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_HEATER_OVER_VOL_ABSOLUTE);
    }
    if (ETMAnalogCheckUnderRelative(&global_data_A36224_500.analog_input_heater_voltage)) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_HEATER_UNDER_VOL_RELATIVE);
    }
    
    
    if (ETMAnalogCheckOverAbsolute(&global_data_A36224_500.analog_input_electromagnet_current)) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_MAGNET_OVER_CUR_ABSOLUTE);
    }
    if (ETMAnalogCheckUnderAbsolute(&global_data_A36224_500.analog_input_electromagnet_current)) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_MAGNET_UNDER_CUR_ABSOLUTE);
    }
    if (ETMAnalogCheckOverRelative(&global_data_A36224_500.analog_input_electromagnet_current)) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_MAGNET_OVER_CUR_RELATIVE); 
    }
    if (ETMAnalogCheckUnderRelative(&global_data_A36224_500.analog_input_electromagnet_current)) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_MAGNET_UNDER_CUR_RELATIVE); 
    }
    if (ETMAnalogCheckOverAbsolute(&global_data_A36224_500.analog_input_electromagnet_voltage)) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_MAGNET_OVER_VOL_ABSOLUTE); 
    }
    if (ETMAnalogCheckUnderRelative(&global_data_A36224_500.analog_input_electromagnet_voltage)) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_BIT_MAGNET_UNDER_VOL_RELATIVE);
    }
    
    // Set DAC outputs
    ETMAnalogScaleCalibrateDACSetting(&global_data_A36224_500.analog_output_heater_current);
    WriteMCP4822(&U42_MCP4822, MCP4822_OUTPUT_A_4096, global_data_A36224_500.analog_output_heater_current.dac_setting_scaled_and_calibrated>>4);
    
    ETMAnalogScaleCalibrateDACSetting(&global_data_A36224_500.analog_output_electromagnet_current);
    WriteMCP4822(&U42_MCP4822, MCP4822_OUTPUT_B_4096, global_data_A36224_500.analog_output_electromagnet_current.dac_setting_scaled_and_calibrated>>4);

  }
}

void InitializeA36224(void) {

  // Initialize the Analog Input * Output Scaling
  // Dparker need to read from EEPROM
  

  global_data_A36224_500.analog_output_electromagnet_current.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(1.6);
  global_data_A36224_500.analog_output_electromagnet_current.fixed_offset                    = 0;
  global_data_A36224_500.analog_output_electromagnet_current.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_500.analog_output_electromagnet_current.calibration_internal_offset     = 0;
  global_data_A36224_500.analog_output_electromagnet_current.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_500.analog_output_electromagnet_current.calibration_external_offset     = 0;

  global_data_A36224_500.analog_output_heater_current.fixed_scale                            = MACRO_DEC_TO_SCALE_FACTOR_16(1.6);
  global_data_A36224_500.analog_output_heater_current.fixed_offset                           = 0;
  global_data_A36224_500.analog_output_heater_current.calibration_internal_scale             = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_500.analog_output_heater_current.calibration_internal_offset            = 0;
  global_data_A36224_500.analog_output_heater_current.calibration_external_scale             = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_500.analog_output_heater_current.calibration_external_offset            = 0;







  global_data_A36224_500.analog_input_electromagnet_current.fixed_scale                      = MACRO_DEC_TO_SCALE_FACTOR_16(1.3107);
  global_data_A36224_500.analog_input_electromagnet_current.fixed_offset                     = 0;
  global_data_A36224_500.analog_input_electromagnet_current.calibration_internal_scale       = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_500.analog_input_electromagnet_current.calibration_internal_offset      = 0;
  global_data_A36224_500.analog_input_electromagnet_current.calibration_external_scale       = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_500.analog_input_electromagnet_current.calibration_external_offset      = 0;

  global_data_A36224_500.analog_input_electromagnet_current.over_trip_point_absolute         = 25000; //25 Amps
  global_data_A36224_500.analog_input_electromagnet_current.under_trip_point_absolute        = 0;     //0 Amps
  global_data_A36224_500.analog_input_electromagnet_current.relative_trip_point_scale        = MACRO_DEC_TO_CAL_FACTOR_2(0);  
  global_data_A36224_500.analog_input_electromagnet_current.relative_trip_point_floor        = 2000; 
  global_data_A36224_500.analog_input_electromagnet_current.relative_counter_fault_limit     = 30;    // Test occurs at 10mS per test


  global_data_A36224_500.analog_input_heater_current.fixed_scale                             = MACRO_DEC_TO_SCALE_FACTOR_16(1.3107);
  global_data_A36224_500.analog_input_heater_current.fixed_offset                            = 0;
  global_data_A36224_500.analog_input_heater_current.calibration_internal_scale              = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_500.analog_input_heater_current.calibration_internal_offset             = 0;
  global_data_A36224_500.analog_input_heater_current.calibration_external_scale              = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_500.analog_input_heater_current.calibration_external_offset             = 0;
  // DPARKER add the trip points settings for all of the analog inputs

  global_data_A36224_500.analog_input_electromagnet_voltage.fixed_scale                      = MACRO_DEC_TO_SCALE_FACTOR_16(2.6085);
  global_data_A36224_500.analog_input_electromagnet_voltage.fixed_offset                     = 0;
  global_data_A36224_500.analog_input_electromagnet_voltage.calibration_internal_scale       = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_500.analog_input_electromagnet_voltage.calibration_internal_offset      = 0;
  global_data_A36224_500.analog_input_electromagnet_voltage.calibration_external_scale       = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_500.analog_input_electromagnet_voltage.calibration_external_offset      = 0;


  global_data_A36224_500.analog_input_heater_voltage.fixed_scale                             = MACRO_DEC_TO_SCALE_FACTOR_16(2.6085);
  global_data_A36224_500.analog_input_heater_voltage.fixed_offset                            = 0;
  global_data_A36224_500.analog_input_heater_voltage.calibration_internal_scale              = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_500.analog_input_heater_voltage.calibration_internal_offset             = 0;
  global_data_A36224_500.analog_input_heater_voltage.calibration_external_scale              = MACRO_DEC_TO_CAL_FACTOR_2(1);
  global_data_A36224_500.analog_input_heater_voltage.calibration_external_offset             = 0;

    
  etm_can_status_register.status_word_0 = 0x0000;
  etm_can_status_register.status_word_1 = 0x0000;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000; 
  etm_can_status_register.status_word_0_inhbit_mask = 0b0000000100000100;  // DPARKER move this to #define somewhere
  etm_can_status_register.status_word_1_fault_mask  = 0b0001111111111111;  // DParker move this to #define somewhere


  global_data_A36224_500.analog_output_electromagnet_current.set_point = 0;
  global_data_A36224_500.analog_output_heater_current.set_point = 0;

  global_data_A36224_500.analog_input_electromagnet_current.adc_accumulator = 0;
  global_data_A36224_500.analog_input_electromagnet_current.filtered_adc_reading = 0;
  global_data_A36224_500.analog_input_electromagnet_voltage.adc_accumulator = 0;
  global_data_A36224_500.analog_input_electromagnet_voltage.filtered_adc_reading = 0;
  global_data_A36224_500.analog_input_heater_current.adc_accumulator = 0;
  global_data_A36224_500.analog_input_heater_current.filtered_adc_reading = 0;
  global_data_A36224_500.analog_input_heater_voltage.adc_accumulator = 0;
  global_data_A36224_500.analog_input_heater_voltage.filtered_adc_reading = 0;

  global_data_A36224_500.accumulator_counter = 0;
  


  // Configure Inhibit Interrupt
  _INT3IP = 7; // This must be the highest priority interrupt
  _INT1EP = 0; // Positive Transition
  
  // Configure ADC Interrupt
  _ADIP   = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)
  
  
  // Initialize all I/O Registers
  TRISA = A36444_TRISA_VALUE;
  TRISB = A36444_TRISB_VALUE;
  TRISC = A36444_TRISC_VALUE;
  TRISD = A36444_TRISD_VALUE;
  TRISF = A36444_TRISF_VALUE;
  TRISG = A36444_TRISG_VALUE;


  /*
    What is T1 used for anyways???


    // Initialize TMR1
    T1CON = T1CON_VALUE;
    TMR1  = 0;
    _T1IF = 0;
    
  */ 

  // Initialize TMR5
  T5CON = T5CON_VALUE;
  TMR5  = 0;
  _T5IF = 0;
  PR5   = PR5_VALUE_10_MILLISECONDS;
  
  // Initialize interal ADC
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters
  
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCSSL = ADCSSL_SETTING;             // Set which analog pins are scanned
  _ADIF = 0;
  _ADIE = 1;
  _ADON = 1;

  

  // Initialize Both MCP4822 DACs
  U42_MCP4822.pin_chip_select_not = _PIN_RD14;
  U42_MCP4822.pin_load_dac_not = _PIN_RF5;
  U42_MCP4822.spi_port = ETM_SPI_PORT_2;
  U42_MCP4822.spi_con1_value = MCP4822_SPI_CON_VALUE;
  U42_MCP4822.spi_con2_value = MCP4822_SPI_CON2_VALUE;
  U42_MCP4822.spi_stat_value = MCP4822_SPI_STAT_VALUE;
  U42_MCP4822.spi_bit_rate = MCP4822_SPI_1_M_BIT;
  U42_MCP4822.fcy_clk = FCY_CLK;

  U44_MCP4822.pin_chip_select_not = _PIN_RD15;
  U44_MCP4822.pin_load_dac_not = _PIN_RF5;
  U44_MCP4822.spi_port = U42_MCP4822.spi_port;
  U44_MCP4822.spi_con1_value = U42_MCP4822.spi_con1_value;
  U44_MCP4822.spi_con2_value = U42_MCP4822.spi_con2_value;
  U44_MCP4822.spi_stat_value = U42_MCP4822.spi_stat_value;
  U44_MCP4822.spi_bit_rate = U42_MCP4822.spi_bit_rate;
  U44_MCP4822.fcy_clk = U42_MCP4822.fcy_clk;

  SetupMCP4822(&U42_MCP4822);
  SetupMCP4822(&U44_MCP4822);

  // Initialize the CAN module
  ETMCanInitialize();


  // Flash LEDs at boot up
  __delay32(1000000);
  ClrWdt();
  PIN_LED_I2_D = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_I2_C = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_I2_B = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_I2_A = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_FLOW = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_COM = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_WATCHDOG = 1;
}



void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;
  
  if (global_data_A36224_500.adc_ignore_current_sample) {
    // There was a pulse durring the sample sequence.  Throw the data away!!!
    global_data_A36224_500.adc_ignore_current_sample = 0;
  } else {
    // Copy Data From Buffer to RAM
    if (_BUFS) {
      // read ADCBUF 0-7
      global_data_A36224_500.analog_input_electromagnet_current.adc_accumulator += ADCBUF0 + ADCBUF4;
      global_data_A36224_500.analog_input_electromagnet_voltage.adc_accumulator += ADCBUF1 + ADCBUF5;
      global_data_A36224_500.analog_input_heater_current.adc_accumulator        += ADCBUF2 + ADCBUF6;
      global_data_A36224_500.analog_input_heater_voltage.adc_accumulator        += ADCBUF3 + ADCBUF7;
    } else {
      // read ADCBUF 8-15
      global_data_A36224_500.analog_input_electromagnet_current.adc_accumulator += ADCBUF8 + ADCBUFC;
      global_data_A36224_500.analog_input_electromagnet_voltage.adc_accumulator += ADCBUF9 + ADCBUFD;
      global_data_A36224_500.analog_input_heater_current.adc_accumulator        += ADCBUFA + ADCBUFE;
      global_data_A36224_500.analog_input_heater_voltage.adc_accumulator        += ADCBUFB + ADCBUFF;
    }
    
    global_data_A36224_500.accumulator_counter += 2;
    
    if (global_data_A36224_500.accumulator_counter >= 256) {

      global_data_A36224_500.analog_input_electromagnet_current.adc_accumulator >>= 4;  // This is now a 16 bit number average of previous 256 samples 
      global_data_A36224_500.analog_input_electromagnet_current.filtered_adc_reading = global_data_A36224_500.analog_input_electromagnet_current.adc_accumulator;
      global_data_A36224_500.analog_input_electromagnet_current.adc_accumulator = 0;

      global_data_A36224_500.analog_input_electromagnet_voltage.adc_accumulator >>= 4;  // This is now a 16 bit number average of previous 256 samples 
      global_data_A36224_500.analog_input_electromagnet_voltage.filtered_adc_reading = global_data_A36224_500.analog_input_electromagnet_voltage.adc_accumulator;
      global_data_A36224_500.analog_input_electromagnet_voltage.adc_accumulator = 0;

      global_data_A36224_500.analog_input_heater_current.adc_accumulator >>= 4;  // This is now a 16 bit number average of previous 256 samples 
      global_data_A36224_500.analog_input_heater_current.filtered_adc_reading = global_data_A36224_500.analog_input_heater_current.adc_accumulator;
      global_data_A36224_500.analog_input_heater_current.adc_accumulator = 0;

      global_data_A36224_500.analog_input_heater_voltage.adc_accumulator >>= 4;  // This is now a 16 bit number average of previous 256 samples 
      global_data_A36224_500.analog_input_heater_voltage.filtered_adc_reading = global_data_A36224_500.analog_input_heater_voltage.adc_accumulator;
      global_data_A36224_500.analog_input_heater_voltage.adc_accumulator = 0;

      global_data_A36224_500.accumulator_counter = 0;
    }
  }
}



void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}
