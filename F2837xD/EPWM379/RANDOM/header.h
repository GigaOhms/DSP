#define EPWM1_TIMER_TBPRD  10000U  // Period register
#define EPWM1_CMPA          3000U
#define EPWM1_CMPB          7000U

__interrupt void epwm1_isr(void);

void InitEPwm1(void);
void setup_gpio(void);
void setup_ADC(void);

__interrupt void cpuTimer0ISR(void);
__interrupt void cpuTimer1ISR(void);
__interrupt void cpuTimer2ISR(void);

void Init_Func(void);


__interrupt void epwm1_isr(void)
{
    // ADC enable
    AdcaRegs.ADCSOCFRC1.bit.SOC1 = 1;   // associated with pin ADCINA1, on INT1 (SOC2 is ADCINA2 on INT 3) (SOC0 is ADCINA0, on INT1)

    // Clear INT flag for this timer
    EPwm1Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}



void setup_gpio(void)
{
    EALLOW;
//**********************configuration for GPIO module/// MUX=1 FOR EPWM ****************
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // GPIO0 = PWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // GPIO1 = PWM1B

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // GPIO2 = PWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // GPIO3 = PWM2B

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // GPIO4 = PWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // GPIO5 = PWM3B

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // GPIO0 = PWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // GPIO1 = PWM4B

    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // GPIO2 = PWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // GPIO3 = PWM5B

    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;  // GPIO4 = PWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;  // GPIO5 = PWM6B

    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // GPIO0 = PWM7A
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // GPIO1 = PWM7B

    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;  // GPIO2 = PWM8A
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;  // GPIO3 = PWM8B

//********************* an GPIO output/input  mux=0 ************************************
//***********************************************************************
//************* DIR=1 is output && DIR=0 is input**************
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;  // GPIO34 = output

    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0; // GPIO34 = GPIO34
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0; // GPIO34 = GPIO34
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0; // GPIO34 = GPIO34
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0; // GPIO34 = GPIO34
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0; // GPIO34 = GPIO34
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0; // GPIO34 = GPIO34
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;  // GPIO34 = output
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0; // GPIO34 = GPIO34
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;  // GPIO34 = output

//********************* Enable pullup************************************
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pullup on GPIO0
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;    // Enable pullup on GPIO1
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pullup on GPIO2
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pullup on GPIO3
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;    // Enable pullup on GPIO4
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pullup on GPIO5
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pullup on GPIO6
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;    // Enable pullup on GPIO7
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;    // Enable pullup on GPIO8
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;    // Enable pullup on GPIO9
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;   // Enable pullup on GPIO10
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;   // Enable pullup on GPIO11
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;   // Enable pullup on GPIO12
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;   // Enable pullup on GPIO13
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;   // Enable pullup on GPIO14
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;   // Enable pullup on GPIO15

    //******************************************************************
    //**********************configuration for GPIO module****************
    EDIS;
}

// -------------------------------------------------------------------------------------
//__interrupt void epwm1_isr(void)
//{
//    AdcaRegs.ADCSOCFRC1.bit.SOC1 = 1;   // associated with pin ADCINA1, on INT1 (ADCINA0-INT0 = 0; ADCINA1-INT1 = 1)
//
//    EPwm1Regs.ETCLR.bit.INT = 1;        // Clear INT flag for this timer
//
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;     // Acknowledge this interrupt to receive more interrupts from group 3
//}
//--------------------------------------------------------------------------------------

void InitEPwm1()
{
    // Setup TBCLK
    EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;       // Set timer period 801 TBCLKs
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter

    // Set Compare values
    EPwm1Regs.CMPA.bit.CMPA = EPWM1_CMPA;    // Set compare A value
    EPwm1Regs.CMPB.bit.CMPB = EPWM1_CMPB;    // Set Compare B value

    // Setup counter mode
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on event A, up count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM1A on event A, down count

    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;            // Set PWM1B on event B, up count
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // Clear PWM1B on event B, down count

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event
}

void setup_ADC(void)
{
    EALLOW;                 // enable ADCA_0 -------------------------------------------------------------------------
    //write configurations
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    // AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;    // Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;      //power up the ADC

    DELAY_US(1000);         //delay for 1ms to allow ADC time to power up

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;  //SOC0 will convert pin A0 (A0 = 0; A1 = 1; ..........)
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 10; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
    EDIS;
}


void Init_Func(void)
{
    InitSysCtrl();
    DINT;           // Disable CPU interrupts
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    // CAll setup function-----------------------------------------------------------------------------------------------------
    InitEPwm1Gpio();
    InitEPwm1();
    setup_ADC();
    setup_gpio();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    InitPieCtrl();  // Initialize the PIE control registers to their default state.

// Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();     // Initialize the PIE vector table with pointers to the shell Interrupt

// ------------------------------------------------ MAP ISR functions ----------------------------------------------------------------
    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &epwm1_isr;
    PieVectTable.TIMER0_INT = &cpuTimer0ISR;
    PieVectTable.TIMER1_INT = &cpuTimer1ISR;
    PieVectTable.TIMER2_INT = &cpuTimer2ISR;
    EDIS;   // This is needed to disable write to EALLOW protected registers

// ----------------------------------------------- end Map ISR functions -------------------------------------------------------------
    InitCpuTimers();   // Cpu Timers.
    ConfigCpuTimer(&CpuTimer0, 200, 1); // interrupt every second: 100MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer1, 200, 1);
    ConfigCpuTimer(&CpuTimer2, 200, 1);

// ----------------------------------------------- Map ISR register ------------------------------------------------------------------
    IER |= M_INT3;                          // Enable CPU INT3  which is connected to EPWM1-3 INT:
    IER |= M_INT1;                          // Enable CPU int1  which is connected to CPU-Timer 0
    IER |= M_INT13;                         // Enable CPU int13 which is connected to CPU-Timer 1
    IER |= M_INT14;                         // Enable CPU int14 which is connected to CPU-Timer 2

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;      // Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // Enable TINT0 in the PIE: Group 1 interrupt 7

// Enable global Interrupts and higher priority real-time debug events: --------------------------------------------------------------
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

//    Enable 200Mhz ---------------------------------------------------------------------------------------------------------------------
//    EALLOW;
//    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0;
//    EDIS;
}

__interrupt void cpuTimer0ISR(void)
{
    // GpioDataRegs.GPATOGGLE.bit.GPIO13=1;
    // The CPU acknowledges the interrupt
    CpuTimer0.InterruptCount++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void cpuTimer1ISR(void)
{
    // GpioDataRegs.GPATOGGLE.bit.GPIO13=1;

    // The CPU acknowledges the interrupt
    CpuTimer1.InterruptCount++;
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void cpuTimer2ISR(void)
{
    //t=period_sine/1000000.0; //Fsw=1M
    //period_sine=period_sine+1;
    //if(period_sine>19999) period_sine=0;


    // The CPU acknowledges the interrupt
    CpuTimer2.InterruptCount++;
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
