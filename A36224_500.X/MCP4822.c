#include "MCP4822.h"

unsigned int MCP4822_error_count = 0;

void SetupMCP4822(MCP4822* ptr_MCP4822) {
  // See h File For Documentation
  
  ETMSetPin(ptr_MCP4822->pin_load_dac_not);
  ETMSetPin(ptr_MCP4822->pin_chip_select_not);
  
  ETMPinTrisOutput(ptr_MCP4822->pin_load_dac_not);  
  ETMPinTrisOutput(ptr_MCP4822->pin_chip_select_not);

  ConfigureSPI(ptr_MCP4822->spi_port, ptr_MCP4822->spi_con1_value, ptr_MCP4822->spi_con2_value, ptr_MCP4822->spi_stat_value, ptr_MCP4822->spi_bit_rate, ptr_MCP4822->fcy_clk);
}



unsigned char WriteMCP4822(MCP4822* ptr_MCP4822, unsigned int command_word, unsigned int data_12_bit) {
  // See h File For Documentation
  
  unsigned char spi_error;
  unsigned long temp;
  
  data_12_bit &= 0x0FFF;
  command_word &= 0xF000;
  command_word |= data_12_bit;

  spi_error = 0;
  
  ETMClearPin(ptr_MCP4822->pin_chip_select_not);

  temp = SendAndReceiveSPI(command_word, ptr_MCP4822->spi_port);
  if (temp == 0x11110000) {
    spi_error = 0b00000001;
  } 

  ETMSetPin(ptr_MCP4822->pin_chip_select_not);

  ETMClearPin(ptr_MCP4822->pin_load_dac_not);
  __asm__ ("nop");
  __asm__ ("nop");
  __asm__ ("nop");
  __asm__ ("nop");
  ETMSetPin(ptr_MCP4822->pin_load_dac_not);

  if (spi_error != 0) {
    MCP4822_error_count++;
  }
  return spi_error;
}
