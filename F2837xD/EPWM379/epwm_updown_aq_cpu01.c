#include "F28x_Project.h"
#include "./RANDOM/header.h"

int a;
void main(void)
{
    Init_Func();

    for(;;)
    {
        a=AdcaResultRegs.ADCRESULT1;
        EPwm1Regs.CMPA.bit.CMPA=a/4095*5000;
        asm ("    NOP");
    }
}
