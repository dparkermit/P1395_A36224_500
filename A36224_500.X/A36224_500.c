#include "A36224_500.h"
#include "ETM_CAN_PUBLIC.h"
#include "MCP4822.h"
#include "ETM_SCALE.h"




void Do10msTicToc(void);





typedef struct {
  unsigned int filtered_adc_reading;
  unsigned int reading_scaled_and_calibrated;

  // -------- These are used to calibrate and scale the ADC Reading to Engineering Units ---------
  unsigned int fixed_scale;
  signed int   fixed_offset;
  unsigned int calibration_internal_scale;
  signed int   calibration_internal_offset;
  unsigned int calibration_external_scale;
  signed int   calibration_external_offset;


  // --------  These are used for fault detection ------------------ 
  unsigned int over_trip_point_absolute;          // If the value exceeds this it will trip immediatly
  unsigned int under_trip_point_absolute;         // If the value is less than this it will trip immediatly
  unsigned int target_value;                      // This is the target value (probably set point) in engineering units
  unsigned int relative_trip_point_scale;         // This will be something like 5%, 10%, ect
  unsigned int relative_trip_point_floor;         // If target_value * relative_trip_point_floor is less than the floor value, the floor value will be used instead
  unsigned int relative_over_trip_point;          // This is = [target_value + target_value*relative_trip_point_scale] or [target_value + relative_over_trip_point] 
  unsigned int relative_under_trip_point;         // This is = [target_value - target_value*relative_trip_point_scale] or [target_value - relative_over_trip_point] 
  unsigned int over_trip_counter;                 // This counts the number of samples over the relative_over_trip_point (will decrement each sample over test is false)
  unsigned int under_trip_counter;                // This counts the number of samples under the relative_under_trip_point (will decrement each sample under test is false)

} AnalogInput;



typedef struct {
  unsigned int set_point;
  unsigned int dac_setting_scaled_and_calibrated;

  // -------- These are used to calibrate and scale the ADC Reading to Engineering Units ---------
  unsigned int fixed_scale;
  signed int   fixed_offset;
  unsigned int calibration_internal_scale;
  signed int   calibration_internal_offset;
  unsigned int calibration_external_scale;
  signed int   calibration_external_offset;

} AnalogOutput;



void ETMScaleCalibrateDACSetting(AnalogOutput* ptr_analog_output);
void ETMScaleCalibrateADCReading(AnalogInput* ptr_analog_input);

void ETMScaleCalibrateDACSetting(AnalogOutput* ptr_analog_output) {
  unsigned int temp;
  // Convert from engineering units to the DAC scale
  temp = ETMScaleFactor16(ptr_analog_output->set_point, ptr_analog_output->fixed_scale, ptr_analog_output->fixed_offset);
  
  // Calibrate for known gain/offset errors of this board
  temp = ETMScaleFactor2(temp, ptr_analog_output->calibration_internal_scale, ptr_analog_output->calibration_internal_offset);

  // Calibrate the DAC output for known gain/offset errors of the external circuitry
  temp = ETMScaleFactor2(temp, ptr_analog_output->calibration_external_scale, ptr_analog_output->calibration_external_offset);
  
  ptr_analog_output->dac_setting_scaled_and_calibrated = temp;
}

void ETMScaleCalibrateADCReading(AnalogInput* ptr_analog_input) {
  unsigned int temp;
  // Calibrate the adc reading based on the known gain/offset errors of the external circuitry
  temp = ETMScaleFactor16(ptr_analog_input->filtered_adc_reading, ptr_analog_input->calibration_external_scale, ptr_analog_input->calibration_external_offset);

  // Calibrate the adc reading based on the known gain/offset errors of this board
  temp = ETMScaleFactor2(temp, ptr_analog_input->calibration_internal_scale, ptr_analog_input->calibration_internal_offset);
  
  // Scale the analog input to engineering units based on the fixed scale (and offset but normally not required) for this application
  temp = ETMScaleFactor16(temp, ptr_analog_input->fixed_scale, ptr_analog_input->fixed_offset);

  ptr_analog_input->reading_scaled_and_calibrated = temp;
}

 
AnalogInput analog_input_heater_voltage;
AnalogInput analog_input_heater_current;

AnalogInput analog_input_electromagnet_voltage;
AnalogInput analog_input_electromagnet_current;

AnalogOutput analog_output_heater_current;
AnalogOutput analog_output_electromagnet_current;



HeaterMagnetControlData global_data_A36224_500;

// This is firmware for the HV Lambda Board

_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer 
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


void InitializeA36224(void);


MCP4822 U42_MCP4822;
MCP4822 U44_MCP4822;


int main(void) {
  InitializeA36224();

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
  __delay32(1000000);
  ClrWdt();


  while (1) {
    ETMCanDoCan();
    // This Manages all the Can related functions, it should be called every time through the control loop
    // It calls ETMCanProcessMessage, All of the timed data transmits, Processes log data, ect
    
    Do10msTicToc();      

  }
}


unsigned int led_divider;

void Do10msTicToc(void) {   // DPARKER need beter name
  if (_T5IF) {
    // 10ms Timer has expired
    _T5IF = 0;
       
    // Do Math on ADC inputs
    
    // Look for faults
    
    // Set DAC outputs
    
    
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

    
    analog_output_heater_current.set_point = global_data_A36224_500.heater_current_set_point;
    ETMScaleCalibrateDACSetting(&analog_output_heater_current);
    WriteMCP4822(&U42_MCP4822, MCP4822_OUTPUT_A_4096, analog_output_heater_current.dac_setting_scaled_and_calibrated>>4);
    
    analog_output_electromagnet_current.set_point = global_data_A36224_500.magnet_current_set_point;
    if (analog_output_electromagnet_current.set_point >= 10000) {
      analog_output_electromagnet_current.set_point = 0;
    }
    ETMScaleCalibrateDACSetting(&analog_output_electromagnet_current);
    WriteMCP4822(&U42_MCP4822, MCP4822_OUTPUT_B_4096, analog_output_electromagnet_current.dac_setting_scaled_and_calibrated>>4);
    
    
  }

}

void InitializeA36224(void) {

  // Initialize the Analog Input * Output Scaling
  // Dparker need to read from EEPROM
  

  analog_output_electromagnet_current.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(1.6);
  analog_output_electromagnet_current.fixed_offset                    = 0;
  analog_output_electromagnet_current.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  analog_output_electromagnet_current.calibration_internal_offset     = 0;
  analog_output_electromagnet_current.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
  analog_output_electromagnet_current.calibration_external_offset     = 0;

  analog_output_heater_current.fixed_scale                            = MACRO_DEC_TO_SCALE_FACTOR_16(1.6);
  analog_output_heater_current.fixed_offset                           = 0;
  analog_output_heater_current.calibration_internal_scale             = MACRO_DEC_TO_CAL_FACTOR_2(1);
  analog_output_heater_current.calibration_internal_offset            = 0;
  analog_output_heater_current.calibration_external_scale             = MACRO_DEC_TO_CAL_FACTOR_2(1);
  analog_output_heater_current.calibration_external_offset            = 0;

  analog_input_electromagnet_current.fixed_scale                      = MACRO_DEC_TO_SCALE_FACTOR_16(1.3107);
  analog_input_electromagnet_current.fixed_offset                     = 0;
  analog_input_electromagnet_current.calibration_internal_scale       = MACRO_DEC_TO_CAL_FACTOR_2(1);
  analog_input_electromagnet_current.calibration_internal_offset      = 0;
  analog_input_electromagnet_current.calibration_external_scale       = MACRO_DEC_TO_CAL_FACTOR_2(1);
  analog_input_electromagnet_current.calibration_external_offset      = 0;

  analog_input_heater_current.fixed_scale                             = MACRO_DEC_TO_SCALE_FACTOR_16(1.3107);
  analog_input_heater_current.fixed_offset                            = 0;
  analog_input_heater_current.calibration_internal_scale              = MACRO_DEC_TO_CAL_FACTOR_2(1);
  analog_input_heater_current.calibration_internal_offset             = 0;
  analog_input_heater_current.calibration_external_scale              = MACRO_DEC_TO_CAL_FACTOR_2(1);
  analog_input_heater_current.calibration_external_offset             = 0;


  analog_input_electromagnet_voltage.fixed_scale                      = MACRO_DEC_TO_SCALE_FACTOR_16(2.6085);
  analog_input_electromagnet_voltage.fixed_offset                     = 0;
  analog_input_electromagnet_voltage.calibration_internal_scale       = MACRO_DEC_TO_CAL_FACTOR_2(1);
  analog_input_electromagnet_voltage.calibration_internal_offset      = 0;
  analog_input_electromagnet_voltage.calibration_external_scale       = MACRO_DEC_TO_CAL_FACTOR_2(1);
  analog_input_electromagnet_voltage.calibration_external_offset      = 0;

  analog_input_heater_voltage.fixed_scale                             = MACRO_DEC_TO_SCALE_FACTOR_16(2.6085);
  analog_input_heater_voltage.fixed_offset                            = 0;
  analog_input_heater_voltage.calibration_internal_scale              = MACRO_DEC_TO_CAL_FACTOR_2(1);
  analog_input_heater_voltage.calibration_internal_offset             = 0;
  analog_input_heater_voltage.calibration_external_scale              = MACRO_DEC_TO_CAL_FACTOR_2(1);
  analog_input_heater_voltage.calibration_external_offset             = 0;
    
  etm_can_status_register.status_word_0 = 0x0000;
  etm_can_status_register.status_word_1 = 0x0000;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000; 

  global_data_A36224_500.magnet_current_set_point = 4000;  // DPARKER change back to zero
  global_data_A36224_500.heater_current_set_point = 4000;  // DPARKER change back to zero 
  global_data_A36224_500.magnet_current_dac_reading = 0;
  global_data_A36224_500.magnet_voltage_dac_reading = 0;
  global_data_A36224_500.heater_current_dac_reading = 0;
  global_data_A36224_500.heater_voltage_dac_reading = 0;
  global_data_A36224_500.magnet_current_dac_accumulator = 0;
  global_data_A36224_500.magnet_voltage_dac_accumulator = 0;
  global_data_A36224_500.heater_current_dac_accumulator = 0;
  global_data_A36224_500.heater_voltage_dac_accumulator = 0;
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
      global_data_A36224_500.magnet_current_dac_accumulator += ADCBUF0 + ADCBUF4;
      global_data_A36224_500.magnet_voltage_dac_accumulator += ADCBUF1 + ADCBUF5;
      global_data_A36224_500.heater_current_dac_accumulator += ADCBUF2 + ADCBUF6;
      global_data_A36224_500.heater_voltage_dac_accumulator += ADCBUF3 + ADCBUF7;
    } else {
      // read ADCBUF 8-15
      global_data_A36224_500.magnet_current_dac_accumulator += ADCBUF8 + ADCBUFC;
      global_data_A36224_500.magnet_voltage_dac_accumulator += ADCBUF9 + ADCBUFD;
      global_data_A36224_500.heater_current_dac_accumulator += ADCBUFA + ADCBUFE;
      global_data_A36224_500.heater_voltage_dac_accumulator += ADCBUFB + ADCBUFF;
    }
    
    global_data_A36224_500.accumulator_counter += 2;
    
    if (global_data_A36224_500.accumulator_counter >= 256) {

      global_data_A36224_500.magnet_current_dac_accumulator >>= 4;  // This is now a 16 bit number average of previous 256 samples 
      global_data_A36224_500.magnet_current_dac_reading = global_data_A36224_500.magnet_current_dac_accumulator;
      global_data_A36224_500.magnet_current_dac_accumulator = 0;

      global_data_A36224_500.magnet_voltage_dac_accumulator >>= 4;  // This is now a 16 bit number average of previous 256 samples 
      global_data_A36224_500.magnet_voltage_dac_reading = global_data_A36224_500.magnet_voltage_dac_accumulator;
      global_data_A36224_500.magnet_voltage_dac_accumulator = 0;

      global_data_A36224_500.heater_current_dac_accumulator >>= 4;  // This is now a 16 bit number average of previous 256 samples 
      global_data_A36224_500.heater_current_dac_reading = global_data_A36224_500.heater_current_dac_accumulator;
      global_data_A36224_500.heater_current_dac_accumulator = 0;

      global_data_A36224_500.heater_voltage_dac_accumulator >>= 4;  // This is now a 16 bit number average of previous 256 samples 
      global_data_A36224_500.heater_voltage_dac_reading = global_data_A36224_500.heater_voltage_dac_accumulator;
      global_data_A36224_500.heater_voltage_dac_accumulator = 0;

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
