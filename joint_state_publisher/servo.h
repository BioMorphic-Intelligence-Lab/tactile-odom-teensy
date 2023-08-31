#include <Arduino.h>

void InitServos(void);

void Serial5EnableOpenDrain(bool bEnable);

void SendInstructionServo4(byte u8_ServoID,
                           byte u8_Instruction,
                           byte* u8_Params,
                           int s32_ParamCount);

uint16_t read_current_pos(int SERVO_ID);
uint16_t read_current_vel(int SERVO_ID);
