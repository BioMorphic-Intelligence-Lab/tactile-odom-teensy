#include "servo.h"

#define INST_READ 0x02
#define CURRENT_POS_ADR  56 // 0x38
#define CURRENT_VEL_ADR  58 // 0x3A
#define TIME_OF_SERVO_TX 150 //[uS] Margin of time needed to avoid TX conflicts when the servos are killed


#define BAUDRATE_SERVO  1000000 //Baudrate for the servo communication.
#define SERVO_serial Serial5
#define SERVO_serialEnableOpenDrain Serial5EnableOpenDrain

IMXRT_LPUART_t *s_pkuart_5 = &IMXRT_LPUART8;  // underlying hardware for Serial5
volatile int position_feedback_servo_lost = 0;
int ack_write_read = 0;
int servo_write_read_lock = 0; 
int iter_counter_SERVO = 0;

// Initialize all the serials of the servos including the tristate mode and PWM servos
void InitServos(void){
  
  SERVO_serial.begin(BAUDRATE_SERVO); //5,6,7,8 on the same serial, not needed to recall them 
  SERVO_serialEnableOpenDrain(true);     

  // Serial 5 half duplex mode:
  s_pkuart_5->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC; //Servos 1,2,3,4 on the same serial, not needed to recall it 
        
}



void Serial5EnableOpenDrain(bool bEnable){
    if (bEnable){
      Serial5.flush();  // Make sure we output everything first before changing state.
      s_pkuart_5->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
    }
    else{
      s_pkuart_5->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
    }
}


void SendInstructionServo(byte u8_ServoID,
                          byte u8_Instruction,
                          byte* u8_Params,
                          int s32_ParamCount){
    // -------- SEND INSTRUCTION ------------   
    int buffer_tx_idx = 0; 
    byte buffer_tx[50];
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = u8_ServoID;
    buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
    buffer_tx[buffer_tx_idx++] = u8_Instruction;
    
    for (int i=0; i<s32_ParamCount; i++)
    {
        buffer_tx[buffer_tx_idx++] = u8_Params[i];
    }

    byte u8_Checksum = 0;
    for (int i=2; i<buffer_tx_idx; i++)
    {
        u8_Checksum += buffer_tx[i];
    }
    buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

    //Send the instruction to servo
    SERVO_serial.clear();
    SERVO_serialEnableOpenDrain(false);
    SERVO_serial.write(buffer_tx, buffer_tx_idx);
    SERVO_serialEnableOpenDrain(true);
    
    #ifdef VERBOSE_MESSAGE
      PrintHex("Instruction sent servo 4: ", buffer_tx, buffer_tx_idx);
    #endif
}

uint16_t read_two_bytes_register(int SERVO_ID, int REGISTER_ADR)
{
  uint16_t result = 0;
  byte u8_Data_Send[2] = {0, 0};
  u8_Data_Send[0] = REGISTER_ADR;
  u8_Data_Send[1] = 2; 

  SendInstructionServo(SERVO_ID, INST_READ,
                          u8_Data_Send,
                          sizeof(u8_Data_Send));
  delayMicroseconds(TIME_OF_SERVO_TX);

  byte buffer_servo[50] = { 0 };
  uint8_t buffer_servo_idx = 0;

  while(SERVO_serial.available())
  {
      buffer_servo[buffer_servo_idx] = SERVO_serial.read();
      buffer_servo_idx++;
  }

  uint8_t bitsum_servo = 0;
  for(int i = 2; i < 7; i++)
  { //Sum the received bits
      bitsum_servo += buffer_servo[i];
  }

  if(255 - bitsum_servo == buffer_servo[7] // Ensure Checksum Correctness
      && buffer_servo[2] == SERVO_ID)      // Ensure Correct Servo ID
  {
      result = buffer_servo[5] + (buffer_servo[6] << 8); 
  }
  else
  {
      position_feedback_servo_lost++;
  }     

  return result;
}

uint16_t read_current_pos(int SERVO_ID)
{
  return read_two_bytes_register(SERVO_ID, CURRENT_POS_ADR);
}

uint16_t read_current_vel(int SERVO_ID)
{
  return read_two_bytes_register(SERVO_ID, CURRENT_VEL_ADR);
}