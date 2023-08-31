#include <Arduino.h>

void InitServos(void);

void Serial5EnableOpenDrain(bool bEnable);

void SendInstructionServo4(byte u8_ServoID,
                           byte u8_Instruction,
                           byte* u8_Params,
                           int s32_ParamCount);

uint16_t ReadCurrentPosition(int SERVO_ID);
