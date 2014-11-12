#include "ETM_ANALOG.h"




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
