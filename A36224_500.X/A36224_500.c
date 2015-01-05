#include "A36224_500.h"
#include "A36224_500_SETTINGS.h"
#include "MCP4822.h"
#include "FIRMWARE_VERSION.h"

_FOSC(ECIO & CSW_FSCM_OFF); 
//_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);



void DisableHeaterMagnetOutputs(void);
void EnableHeaterMagnetOutputs(void);




void InitializeA36224(void);
void DoA36224_500(void);
void DoStateMachine(void);

MCP4822 U42_MCP4822;
MCP4822 U44_MCP4822;

HeaterMagnetControlData global_data_A36224_500;

unsigned int led_divider;


unsigned int control_state;

#define STATE_STARTUP                0x10
#define STATE_WAITING_FOR_CONFIG     0x20
#define STATE_POWER_UP_TEST          0x30
#define STATE_OPERATE                0x40
#define STATE_FAULT                  0x50
#define STATE_FAULT_NO_RECOVERY      0x60

int main(void) {



  control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}





#define TIME_POWER_UP_TEST     1000 // 10 seconds
#define MAX_RESET_ATTEMPTS          5

void DoStateMachine(void) {

  switch (control_state) {
    
  case STATE_STARTUP:
    InitializeA36224();
    _CONTROL_STATE_NOT_OPERATE = 1;
    _CONTROL_NOT_CONFIGURED = 1;
    global_data_A36224_500.startup_count = 0;
    control_state = STATE_WAITING_FOR_CONFIG;
    break;

    
  case STATE_WAITING_FOR_CONFIG:
    _CONTROL_STATE_NOT_OPERATE = 1;
    DisableHeaterMagnetOutputs();
    while (control_state == STATE_WAITING_FOR_CONFIG) {
      DoA36224_500();
      ETMCanDoCan();
    
      if (_CONTROL_NOT_CONFIGURED == 0) {
	control_state = STATE_POWER_UP_TEST;
      }
    }
    break;

    
  case STATE_POWER_UP_TEST:
    global_data_A36224_500.startup_count++;
    _CONTROL_STATE_NOT_OPERATE = 1;
    global_data_A36224_500.power_up_test_timer = 0;
    EnableHeaterMagnetOutputs();
    while (control_state == STATE_POWER_UP_TEST) {
      DoA36224_500();
      ETMCanDoCan();
  
      if (global_data_A36224_500.power_up_test_timer >= TIME_POWER_UP_TEST) {
	control_state = STATE_OPERATE;
      }
      
      if (global_data_A36224_500.fault_active) {
	if (global_data_A36224_500.startup_count <= MAX_RESET_ATTEMPTS){
	  control_state = STATE_FAULT;
	} else {
	  control_state = STATE_FAULT_NO_RECOVERY;
	}
      }
    }
    break;


  case STATE_OPERATE:
    global_data_A36224_500.startup_count = 0;
    _CONTROL_STATE_NOT_OPERATE = 0;
    _FAULT_REGISTER = 0;
    while (control_state == STATE_OPERATE) {
      DoA36224_500();
      ETMCanDoCan();
      
      if (global_data_A36224_500.fault_active) {
	control_state = STATE_FAULT;
      }
    }
    break;


  case STATE_FAULT:
    DisableHeaterMagnetOutputs();
    _CONTROL_STATE_NOT_OPERATE = 1;
    while (control_state == STATE_FAULT) {
      DoA36224_500();
      ETMCanDoCan();
      if (!global_data_A36224_500.fault_active) {
	control_state = STATE_WAITING_FOR_CONFIG;
      }
    }
    break;
    
  case STATE_FAULT_NO_RECOVERY:
    DisableHeaterMagnetOutputs();
    _CONTROL_STATE_NOT_OPERATE = 1;
    while (control_state == STATE_FAULT_NO_RECOVERY) {
      DoA36224_500();
      ETMCanDoCan();
    }
    
  default:
    control_state = STATE_FAULT_NO_RECOVERY;
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



void DoA36224_500(void) {

  // Check the status of these pins every time through the loop
  if (PIN_D_IN_3_HEATER_OVER_VOLT_STATUS == ILL_HEATER_OV) {
    _FAULT_HW_HEATER_OVER_VOLTAGE = 1;
    global_data_A36224_500.fault_active = 1;
  }
  
  if (PIN_D_IN_4_TEMPERATURE_STATUS == ILL_TEMP_SWITCH_FAULT) {
    _FAULT_HW_TEMPERATURE_SWITCH = 1;
    global_data_A36224_500.fault_active = 1;
  }
  

  if (_T5IF) {
    local_debug_data.debug_0 = global_data_A36224_500.startup_count;
    local_debug_data.debug_1 = global_data_A36224_500.fault_active;
    local_debug_data.debug_2 = global_data_A36224_500.power_up_test_timer;
    local_debug_data.debug_2 = control_state;
      

    // 10ms Timer has expired so this code will executre once every 10ms
    _T5IF = 0;
    

    global_data_A36224_500.power_up_test_timer++;
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

    // Update the error counters that get returned
    local_debug_data.i2c_bus_error_count = 0;  // There are no I2C devices on this board
    local_debug_data.spi_bus_error_count = etm_spi1_error_count + etm_spi2_error_count;
    local_debug_data.scale_error_count = etm_scale_saturation_etmscalefactor2_count + etm_scale_saturation_etmscalefactor16_count;
    local_debug_data.self_test_result_register = 0; // DPARKER NEED TO WORK ON THE SELF TEST
    
    /*
      The following are updated by the ETM_CAN module
      can_bus_error_count
      reset_count
    */

    
    // DPARKER FIX THIS
    // Set the fault LED
    if (_CONTROL_NOT_READY) {
      // The board is faulted or inhibiting the system
      PIN_LED_I2_C = 0;
    } else {
      PIN_LED_I2_C = 1;
    }

    // Update the digital input status pins
    if (PIN_D_IN_0_ELECTROMAGENT_STATUS == ILL_POWER_SUPPLY_DISABLED) {
      _STATUS_MAGNET_OFF_READBACK = 1;
    } else {
      _STATUS_MAGNET_OFF_READBACK = 0;
    }

    if (PIN_D_IN_1_HEATER_STATUS == ILL_POWER_SUPPLY_DISABLED) {
      _STATUS_HEATER_OFF_READBACK = 1;
    } else {
      _STATUS_HEATER_OFF_READBACK = 0;
    }
    
    if (PIN_D_IN_5_RELAY_STATUS == ILL_RELAY_OPEN) {
      _STATUS_OUTPUT_RELAY_OPEN = 1;
    } else {
      _STATUS_OUTPUT_RELAY_OPEN = 0;
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
    

// -------------------- CHECK FOR FAULTS ------------------- //

    global_data_A36224_500.fault_active = 0;
    
    // Check the status of these pins every time through the loop
    if (PIN_D_IN_3_HEATER_OVER_VOLT_STATUS == ILL_HEATER_OV) {
      _FAULT_HW_HEATER_OVER_VOLTAGE = 1;
      global_data_A36224_500.fault_active = 1;
    }
    
    if (PIN_D_IN_4_TEMPERATURE_STATUS == ILL_TEMP_SWITCH_FAULT) {
      _FAULT_HW_TEMPERATURE_SWITCH = 1;
      global_data_A36224_500.fault_active = 1;
    }
    
    // DPARKER check SYNC message for coolant flow and fault if there is a problem
    if (0) {
      _FAULT_COOLANT_FAULT = 1;
      global_data_A36224_500.fault_active = 1;
    } else {
      if (global_reset_faults) {
	_FAULT_COOLANT_FAULT = 0;
      }
    }

    if (_CONTROL_CAN_FAULT) {
      global_data_A36224_500.fault_active = 1;
    }

    if ((control_state == STATE_OPERATE) || (control_state == STATE_POWER_UP_TEST)) {
      global_data_A36224_500.analog_input_electromagnet_current.target_value = global_data_A36224_500.analog_output_electromagnet_current.set_point;
      global_data_A36224_500.analog_input_electromagnet_voltage.target_value = ETMScaleFactor16(global_data_A36224_500.analog_output_electromagnet_current.set_point,MACRO_DEC_TO_SCALE_FACTOR_16(NOMINAL_ELECTROMAGNET_RESISTANCE),0);
      global_data_A36224_500.analog_input_heater_current.target_value = global_data_A36224_500.analog_output_heater_current.set_point;
      global_data_A36224_500.analog_input_heater_voltage.target_value = ETMScaleFactor16(global_data_A36224_500.analog_output_heater_current.set_point,MACRO_DEC_TO_SCALE_FACTOR_16(NOMINAL_HEATER_RESISTANCE),0);
      
      if (ETMAnalogCheckOverAbsolute(&global_data_A36224_500.analog_input_heater_current)) {
	_FAULT_HEATER_OVER_CURRENT_ABSOLUTE = 1;
	global_data_A36224_500.fault_active = 1;
      }
      if (ETMAnalogCheckUnderAbsolute(&global_data_A36224_500.analog_input_heater_current)) {
	_FAULT_HEATER_UNDER_CURRENT_ABSOLUTE = 1;
	global_data_A36224_500.fault_active = 1;
      }
      if (ETMAnalogCheckOverRelative(&global_data_A36224_500.analog_input_heater_current)) {
	_FAULT_HEATER_OVER_CURRENT_RELATIVE = 1;
	global_data_A36224_500.fault_active = 1;
      }
      if (ETMAnalogCheckUnderRelative(&global_data_A36224_500.analog_input_heater_current)) {
	_FAULT_HEATER_UNDER_CURRENT_RELATIVE = 1;
	global_data_A36224_500.fault_active = 1;
      }
      if (ETMAnalogCheckOverAbsolute(&global_data_A36224_500.analog_input_heater_voltage)) {
	_FAULT_HEATER_OVER_VOLTAGE_ABSOLUTE = 1;
	global_data_A36224_500.fault_active = 1;
      }
      if (ETMAnalogCheckUnderRelative(&global_data_A36224_500.analog_input_heater_voltage)) {
	_FAULT_HEATER_UNDER_VOLTAGE_RELATIVE = 1;
	global_data_A36224_500.fault_active = 1;
      }
      
      
      if (ETMAnalogCheckOverAbsolute(&global_data_A36224_500.analog_input_electromagnet_current)) {
	_FAULT_MAGNET_OVER_CURRENT_ABSOLUTE = 1;
	global_data_A36224_500.fault_active = 1;
      }
      if (ETMAnalogCheckUnderAbsolute(&global_data_A36224_500.analog_input_electromagnet_current)) {
	_FAULT_MAGNET_UNDER_CURRENT_ABSOLUTE = 1;
	global_data_A36224_500.fault_active = 1;
      }
      if (ETMAnalogCheckOverRelative(&global_data_A36224_500.analog_input_electromagnet_current)) {
	_FAULT_MAGNET_OVER_CURRENT_RELATIVE = 1;
	global_data_A36224_500.fault_active = 1;
      }
      if (ETMAnalogCheckUnderRelative(&global_data_A36224_500.analog_input_electromagnet_current)) {
	_FAULT_MAGNET_UNDER_CURRENT_RELATIVE = 1;
	global_data_A36224_500.fault_active = 1;
      }
      if (ETMAnalogCheckOverAbsolute(&global_data_A36224_500.analog_input_electromagnet_voltage)) {
	_FAULT_MAGNET_OVER_VOLTAGE_ABSOLUTE = 1;
	global_data_A36224_500.fault_active = 1;
      }
      if (ETMAnalogCheckUnderRelative(&global_data_A36224_500.analog_input_electromagnet_voltage)) {
	_FAULT_MAGNET_UNDER_VOLTAGE_RELATIVE = 1;
	global_data_A36224_500.fault_active = 1;
      }
      
    } else {
      global_data_A36224_500.analog_input_electromagnet_current.target_value = 0;
      global_data_A36224_500.analog_input_electromagnet_voltage.target_value = 0;
      global_data_A36224_500.analog_input_heater_current.target_value = 0;
      global_data_A36224_500.analog_input_heater_voltage.target_value = 0;
    }

    
    


    // Set DAC outputs
    if ((control_state == STATE_OPERATE) || (control_state == STATE_POWER_UP_TEST)) {
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36224_500.analog_output_heater_current);
      WriteMCP4822(&U42_MCP4822, MCP4822_OUTPUT_A_4096, global_data_A36224_500.analog_output_electromagnet_current.dac_setting_scaled_and_calibrated>>4);
      
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36224_500.analog_output_electromagnet_current);
      WriteMCP4822(&U42_MCP4822, MCP4822_OUTPUT_B_4096, global_data_A36224_500.analog_output_heater_current.dac_setting_scaled_and_calibrated>>4);
    } else {
      WriteMCP4822(&U42_MCP4822, MCP4822_OUTPUT_A_4096, global_data_A36224_500.analog_output_electromagnet_current.disabled_dac_set_point);
      WriteMCP4822(&U42_MCP4822, MCP4822_OUTPUT_B_4096, global_data_A36224_500.analog_output_heater_current.disabled_dac_set_point);
    }
  }
}

void InitializeA36224(void) {

  etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
  etm_can_my_configuration.firmware_branch    = FIRMWARE_BRANCH;
  etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;
  



  // Initialize the Analog Input * Output Scaling
  // Dparker need to read from EEPROM
  ETMAnalogInitializeOutput(&global_data_A36224_500.analog_output_electromagnet_current, MACRO_DEC_TO_SCALE_FACTOR_16(1.6), OFFSET_ZERO, ANALOG_OUTPUT_NO_CALIBRATION,
			    ELECTROMAGNET_MAX_IPROG, ELECTROMAGNET_MIN_IPROG, 0);

  ETMAnalogInitializeOutput(&global_data_A36224_500.analog_output_heater_current, MACRO_DEC_TO_SCALE_FACTOR_16(1.6), OFFSET_ZERO, ANALOG_OUTPUT_NO_CALIBRATION,
			    HEATER_MAX_IPROG, HEATER_MIN_IPROG, 0);
  
  ETMAnalogInitializeInput(&global_data_A36224_500.analog_input_electromagnet_current, MACRO_DEC_TO_SCALE_FACTOR_16(.7629), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   ELECTROMAGNET_CURRENT_OVER_TRIP, ELECTROMAGNET_CURRENT_UNDER_TRIP, ELECTROMAGNET_CURRENT_RELATIVE_FLOOR, ELECTROMAGNET_CURRENT_RELATIVE_FLOOR, 
			   ELECTROMAGNET_CURRENT_TRIP_TIME);
  
  ETMAnalogInitializeInput(&global_data_A36224_500.analog_input_electromagnet_voltage, MACRO_DEC_TO_SCALE_FACTOR_16(.3834), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   ELECTROMAGNET_VOLTAGE_OVER_TRIP, ELECTROMAGNET_VOLTAGE_UNDER_TRIP, ELECTROMAGNET_VOLTAGE_RELATIVE_FLOOR, ELECTROMAGNET_VOLTAGE_RELATIVE_FLOOR, 
			   ELECTROMAGNET_VOLTAGE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A36224_500.analog_input_heater_current, MACRO_DEC_TO_SCALE_FACTOR_16(.7629), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   HEATER_CURRENT_OVER_TRIP, HEATER_CURRENT_UNDER_TRIP, HEATER_CURRENT_RELATIVE_FLOOR, HEATER_CURRENT_RELATIVE_FLOOR, 
			   HEATER_CURRENT_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A36224_500.analog_input_heater_voltage, MACRO_DEC_TO_SCALE_FACTOR_16(.3834), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   HEATER_VOLTAGE_OVER_TRIP, HEATER_VOLTAGE_UNDER_TRIP, HEATER_VOLTAGE_RELATIVE_FLOOR, HEATER_VOLTAGE_RELATIVE_FLOOR, 
			   HEATER_VOLTAGE_TRIP_TIME);

  _FAULT_REGISTER = 0;
  _CONTROL_REGISTER = 0;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000; 
  etm_can_status_register.unused_A = 0;
  etm_can_status_register.unused_B = 0;


  global_data_A36224_500.analog_output_electromagnet_current.set_point = 0;
  global_data_A36224_500.analog_output_heater_current.set_point = 0;

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
  ADCSSL = 0b0000111100000000;//ADCSSL_SETTING;             // Set which analog pins are scanned
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
      global_data_A36224_500.analog_input_heater_current.adc_accumulator        += ADCBUF1 + ADCBUF5;
      global_data_A36224_500.analog_input_electromagnet_voltage.adc_accumulator += ADCBUF2 + ADCBUF6;
      global_data_A36224_500.analog_input_heater_voltage.adc_accumulator        += ADCBUF3 + ADCBUF7;
    } else {
      // read ADCBUF 8-15
      global_data_A36224_500.analog_input_electromagnet_current.adc_accumulator += ADCBUF8 + ADCBUFC;
      global_data_A36224_500.analog_input_heater_current.adc_accumulator        += ADCBUF9 + ADCBUFD;
      global_data_A36224_500.analog_input_electromagnet_voltage.adc_accumulator += ADCBUFA + ADCBUFE;
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
