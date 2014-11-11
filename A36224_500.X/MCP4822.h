#ifndef __MCP4822_H
#define __MCP4822_H

#include "ETM_SPI.h"
#include "ETM_IO_PORTS.h"


/*
  This module is used to interface with a MCP4822 over the SPI bus.


  Module Requirements
  ETM_SPI
  ETM_IO_PORTS

  Dan Parker
  2014_06_02

*/

/*
  This module has been validated on A34760 (pic30F6014).  This board does not use all of the IO pins so it should be further validated on new designs
*/



typedef struct {
  unsigned long pin_chip_select_not;
  unsigned long pin_load_dac_not;
  unsigned char spi_port;
  unsigned int  spi_con1_value;
  unsigned int  spi_con2_value;
  unsigned int  spi_stat_value;
  unsigned long spi_bit_rate;
  unsigned long fcy_clk;
} MCP4822;


#define MCP4822_SPI_CON_VALUE  0b0000010100111111 // This works for V1 and V2 because common bits are identical and there is no overlap on the non-identical bits
#define MCP4822_SPI_CON2_VALUE 0b0000000000000000 // This works for V1 and V2 because it is ignored in V1
#define MCP4822_SPI_STAT_VALUE 0b1010000000000000 // This works for V1 & V2 because they are identical


#define MCP4822_SPI_10_M_BIT    10000000
#define MCP4822_SPI_5_M_BIT      5000000
#define MCP4822_SPI_2_5_M_BIT    2500000
#define MCP4822_SPI_1_M_BIT      1000000
  
/* -------------- DAC Command Definitions ------------------- */
#define MCP4822_OUTPUT_A                                  0b0000000000000000
#define MCP4822_OUTPUT_B                                  0b1000000000000000

#define MCP4822_GAIN_1X                                   0b0010000000000000  // This sets the output to 2.048 Reference
#define MCP4822_GAIN_2X                                   0b0000000000000000  // This sets the output to 4.096 Reference

#define MCP4822_OUTPUT_DISABLED                           0b0000000000000000  
#define MCP4822_OUTPUT_ENABLED                            0b0001000000000000

/* -------------- END DAC Command Definitions ------------------- */

#define MCP4822_OUTPUT_A_4096           (MCP4822_OUTPUT_A | MCP4822_GAIN_2X | MCP4822_OUTPUT_ENABLED)
#define MCP4822_OUTPUT_B_4096           (MCP4822_OUTPUT_B | MCP4822_GAIN_2X | MCP4822_OUTPUT_ENABLED)

#define MCP4822_OUTPUT_A_DISABLED       (MCP4822_OUTPUT_A | MCP4822_GAIN_2X | MCP4822_OUTPUT_DISABLED)
#define MCP4822_OUTPUT_B_DISABLED       (MCP4822_OUTPUT_B | MCP4822_GAIN_2X | MCP4822_OUTPUT_DISABLED)



void SetupMCP4822(MCP4822* ptr_MCP4822);
/*  
   Function Arguments:
   *ptr_MCP4822 : This is pointer to the structure that defines a particular MCP4822 chip

   This function should be called when somewhere during the startup of the processor.
   This will configure the SPI port (selected by spi_port), the !CS pin, and the !LDAC pin.
*/



unsigned char WriteMCP4822(MCP4822* ptr_MCP4822, unsigned int command_word, unsigned int data_12_bit);

/*  
   Function Arguments:
   *ptr_MCP4822     : This is pointer to the structure that defines a particular MCP4822 chip

   command_word     : This is the command word as defined above.  See the MCP4822 spec sheet for more information
                      This includes 4 bits of configuration information

   data_12_bit      : This contains the 12 bits of data to be written.  This information must be in the 12 LSB.  the 4 Most Significant Bits will be ignored

   This will send a 16 bit command to to the MCP4822.
   NOTE!!! This chip does not support the ability to confirm that data was properly received
   If the data transmission is successful, the function will return 0
   If there is an error with the data transmission (error on SPI bus), then the function will return the error code
   If there are any errors bad data may have been written to the DAC so the calling code should attempt to resend the data
*/

extern unsigned int MCP4822_error_count; // This global variable counts the number of times WriteMCP4822 failed to properly write data.


#endif
