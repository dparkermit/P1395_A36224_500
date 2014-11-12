#ifndef __ETM_CAN_PUBLIC_H
#define __ETM_CAN_PUBLIC_H

#include "ETM_CAN_USER_CONFIG.h"


typedef struct {
  unsigned int status_word_0;
  unsigned int status_word_1;
  unsigned int data_word_A;
  unsigned int data_word_B;
  
  unsigned int status_word_0_inhbit_mask;
  unsigned int status_word_1_fault_mask;

} ETMCanStatusRegister;


typedef struct {
  unsigned int i2c_bus_error_count;
  unsigned int spi_bus_error_count;
  unsigned int can_bus_error_count;
  unsigned int scale_error_count;

  unsigned int reset_count;
  unsigned int self_test_result_register;
  unsigned int reserved_0;
  unsigned int reserved_1;

  unsigned int debug_0;
  unsigned int debug_1;
  unsigned int debug_2;
  unsigned int debug_3;

  unsigned int debug_4;
  unsigned int debug_5;
  unsigned int debug_6;
  unsigned int debug_7;

  unsigned int debug_8;
  unsigned int debug_9;
  unsigned int debug_A;
  unsigned int debug_B;

  unsigned int debug_C;
  unsigned int debug_D;
  unsigned int debug_E;
  unsigned int debug_F;

} ETMCanSystemDebugData;



#define ETM_CAN_HIGH_ENERGY           1
#define ETM_CAN_LOW_ENERGY            0

// Public Variables
extern unsigned int etm_can_next_pulse_level;  // This value will get updated in RAM as when a next pulse level command is received
extern unsigned int etm_can_next_pulse_count;  // This value will get updated in RAM as when a next pulse level command is received

// Public Debug and Status registers
extern ETMCanSystemDebugData etm_can_system_debug_data;  
extern ETMCanStatusRegister  etm_can_status_register;  
/*
  DPARKER provide more description here.  How is it used.  What bits to set and what affect will setting them have
  This is the status register for this board.  Word0 bits (0,1) and Word1 bits (0) are mangaged by the Can module
*/

// Public Functions
void ETMCanDoCan(void);
/*
  This function should be called every time through the processor execution loop (which should be on the order of 10-100s of uS)
  If will do the following
  1) Look for an execute can commands
  2) Look for changes in status bits, update the Fault/Inhibit bits and send out a new status command if nessesary
  3) Send out regularly schedule communication (On slaves this is status update and logging data)
*/

void ETMCanInitialize(void);
/*
  This is called once when the processor starts up to initialize the can bus and all of the can variables
*/

#ifndef __ETM_CAN_MASTER_MODULE
void ETMCanLogCustomPacketC(void);
void ETMCanLogCustomPacketD(void);
void ETMCanLogCustomPacketE(void);
void ETMCanLogCustomPacketF(void);
#endif


#ifdef __A35487
// Only for Pulse Sync Board
void ETMCanPulseSyncSendNextPulseLevel(unsigned int next_pulse_level, unsigned int next_pulse_count);
#endif


#ifdef __A63417
// Only for Ion Pump Board
void ETMCanIonPumpSendTargetCurrentReading(unsigned int target_current_reading, unsigned int energy_level, unsigned int pulse_count);
#endif


#endif
