/*
 * File:   SPWM.c
 * Author: Devanshu
 *
 * Created on August 8, 2017, 12:42 PM
 */
//
// DSPIC30F4011 Configuration Bit Settings
// 'C' source line config statements
// FOSC
//**********************************************************************************************************************************************************
//                                                                 CONFIGURATIONS
//**********************************************************************************************************************************************************
#pragma config FPR = XT_PLL16           // Primary Oscillator Mode (XT w/PLL 16x)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_PWMPIN      // PWM Output Pin Reset (Control with HPOL/LPOL bits)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
//// Use project enums instead of #define for ON and OFF.
//*******************************************************************************************************************************************************
//                                                      HEADER FILES
//******************************************************************************************************************************************************
#include <xc.h>
#include <dsp.h>
//*******************************************************************************************************************************************************
////                                                    MACROS
//*******************************************************************************************************************************************************
typedef signed int SFRAC16;

#define PIDRefrenceVal 545

#define ADC_ON() ADCON1bits.ADON=1
#define ADC_OFF() ADCON1bits.ADON=0
#define ADC_StartSampling() ADCON1bits.SAMP=1

#define EnablePwmInterrupt() IEC2bits.PWMIE=1
#define EnableTimer3Interrupt() _T3IE=1
#define EnableTimer5Interrupt() _T5IE=1

#define Lcd_delay 400

#define T3_Period 1152    //Prescalar 8, 10ms delay=36864
#define T5_Period 1152     //Prescalar 8, 10 ms delay=36864
#define TMR3_StartTimer() T3CONbits.TON=1
#define TMR5_StartTimer() T5CONbits.TON=1
#define TMR3_StopTimer() T3CONbits.TON=0
#define TMR5_StopTimer() T5CONbits.TON=0
#define TMR3_Reload() PR3=1152
#define TMR5_Reload() PR5=1152

#define PWM1_StopPwm() PTCONbits.PTEN=0
#define PWM1_StartPwm() PTCONbits.PTEN=1

#define ANA0_PWM_Control 0
#define ANA1_Op_HighLow 1
#define ANA2_Op_Overcurrent 2 
#define ANA3_Ip_Voltage 3 
#define ANA4_Ip_Current 4
#define ANA5_Batt_Voltage 5
#define ANA6_Batt_Current 6

#define LCD_D7 LATEbits.LATE5
#define LCD_D6 LATEbits.LATE4
#define LCD_D5 LATEbits.LATE3
#define LCD_D4 LATEbits.LATE2
#define LCD_EN LATEbits.LATE0
#define LCD_RS LATFbits.LATF6
//#define LCD_RW LATFbits.LATF6



#define Op_High_Indicator LATDbits.LATD2
#define Op_Low_Indicator LATDbits.LATD0
#define Op_Overcurrent_Indicator LATDbits.LATD3
#define Overall_Fault_Indicator LATBbits.LATB8

#define Mains_High_Indicator LATCbits.LATC13
#define Main_Low_Indicator  LATDbits.LATD1


#define Batt_Low_Indicator LATEbits.LATE4
#define Batt_High_Indicator LATEbits.LATE5

#define Charger_Card LATCbits.LATC14

#define SFloat_To_SFrac16(Float_Value)	\
        ((Float_Value < 0.0) ? (SFRAC16)(32768 * (Float_Value) - 0.5) \
        : (SFRAC16)(32767 * (Float_Value) + 0.5))
//**********************************************************************************************************************************************************************
//                                                GLOBAL VARIABLES
//*****************************************************************************************************************************************************8
const char Transtech[]="TRANSTECH SYSTEM";

const char Fault[]="FAULT";
const char HighOPVoltage[]="HIGH OP VOLTAGE";
const char LowOPVoltage[]="LOW OP VOLTAGE";
const char HighOpCurrent[]="HIGH OP CURRENT";

const char BattVoltage[]="BATTERY VOLTAGE:";
const char BattLow[]="BATTERY LOW";
const char BattCurrent[]="BATTERY CURRENT:";

const char MainsVoltageLow[]="MAINS VOLATGE LOW";
const char MainsVoltageHigh[]="MAINS VOLTAGE HIGH";
const char MainsCurrentHigh[]="MAINS CURRENT HIGH";
const char MainsCurrent[]="MAINS CURRENT:";
const char MainsVoltage[]="MAINS VOLTAGE:";

const char OPVolatge[]="OUTPUT VOLTAGE:";
const char OPCurrent[]="OUTPUT CURRENT:"; 
const char OPFrequency[]="OUTPUT FREQUENCY:";
   volatile static unsigned int i=0,k=0,temp=0,prevtemp=0,result=0;
   volatile static long int DutyCycle=0;
const unsigned int LookUpTable[512]={325, 350, 375, 400, 425, 450, 476, 501, 526, 551, 576, 601, 626, 651, 676, 701, 726, 751, 776, 801, 826, 851, 876, 901, 926, 951, 976, 1000, 1025, 1050, 1075, 1099, 1124, 1149, 1173, 1198, 1222, 1247, 1271, 1296, 1320, 1344, 1369, 1393, 1417, 1441, 1466, 1490, 1514, 1538, 1562, 1586, 1609, 1633, 1657, 1681, 1704, 1728, 1752, 1775, 1799, 1822, 1845, 1869, 1892, 1915, 1938, 1961, 1984, 2007, 2030, 2052, 2075, 2098, 2120, 2143, 2165, 2188, 2210, 2232, 2254, 2276, 2298, 2320, 2342, 2364, 2386, 2407, 2429, 2450, 2472, 2493, 2514, 2535, 2556, 2577, 2598, 2619, 2640, 2660, 2681, 2701, 2722, 2742, 2762, 2782, 2802, 2822, 2842, 2861, 2881, 2901, 2920, 2939, 2958, 2978, 2997, 3015, 3034, 3053, 3071, 3090, 3108, 3127, 3145, 3163, 3181, 3199, 3216, 3234, 3252, 3269, 3286, 3303, 3320, 3337, 3354, 3371, 3388, 3404, 3420, 3437, 3453, 3469, 3485, 3501, 3516, 3532, 3547, 3562, 3578, 3593, 3608, 3622, 3637, 3652, 3666, 3680, 3694, 3709, 3722, 3736, 3750, 3763, 3777, 3790, 3803, 3816, 3829, 3842, 3854, 3867, 3879, 3891, 3903, 3915, 3927, 3939, 3950, 3962, 3973, 3984, 3995, 4006, 4017, 4027, 4037, 4048, 4058, 4068, 4078, 4087, 4097, 4106, 4116, 4125, 4134, 4143, 4151, 4160, 4168, 4176, 4185, 4193, 4200, 4208, 4216, 4223, 4230, 4237, 4244, 4251, 4258, 4264, 4270, 4277, 4283, 4289, 4294, 4300, 4305, 4311, 4316, 4321, 4326, 4330, 4335, 4339, 4343, 4347, 4351, 4355, 4359, 4362, 4366, 4369, 4372, 4375, 4377, 4380, 4382, 4384, 4386, 4388, 4390, 4392, 4393, 4395, 4396, 4397, 4398, 4398, 4399, 4399, 4399, 4400, 4399, 4399, 4399, 4398, 4398, 4397, 4396, 4395, 4393, 4392, 4390, 4388, 4386, 4384, 4382, 4380, 4377, 4375, 4372, 4369, 4366, 4362, 4359, 4355, 4351, 4347, 4343, 4339, 4335, 4330, 4326, 4321, 4316, 4311, 4305, 4300, 4294, 4289, 4283, 4277, 4270, 4264, 4258, 4251, 4244, 4237, 4230, 4223, 4216, 4208, 4200, 4193, 4185, 4176, 4168, 4160, 4151, 4143, 4134, 4125, 4116, 4106, 4097, 4087, 4078, 4068, 4058, 4048, 4037, 4027, 4017, 4006, 3995, 3984, 3973, 3962, 3950, 3939, 3927, 3915, 3903, 3891, 3879, 3867, 3854, 3842, 3829, 3816, 3803, 3790, 3777, 3763, 3750, 3736, 3722, 3709, 3694, 3680, 3666, 3652, 3637, 3622, 3608, 3593, 3578, 3562, 3547, 3532, 3516, 3501, 3485, 3469, 3453, 3437, 3420, 3404, 3388, 3371, 3354, 3337, 3320, 3303, 3286, 3269, 3252, 3234, 3216, 3199, 3181, 3163, 3145, 3127, 3108, 3090, 3071, 3053, 3034, 3015, 2997, 2978, 2958, 2939, 2920, 2901, 2881, 2861, 2842, 2822, 2802, 2782, 2762, 2742, 2722, 2701, 2681, 2660, 2640, 2619, 2598, 2577, 2556, 2535, 2514, 2493, 2472, 2450, 2429, 2407, 2386, 2364, 2342, 2320, 2298, 2276, 2254, 2232, 2210, 2188, 2165, 2143, 2120, 2098, 2075, 2052, 2030, 2007, 1984, 1961, 1938, 1915, 1892, 1869, 1845, 1822, 1799, 1775, 1752, 1728, 1704, 1681, 1657, 1633, 1609, 1586, 1562, 1538, 1514, 1490, 1466, 1441, 1417, 1393, 1369, 1344, 1320, 1296, 1271, 1247, 1222, 1198, 1173, 1149, 1124, 1099, 1075, 1050, 1025, 1000, 976, 951, 926, 901, 876, 851, 826, 801, 776, 751, 726, 701, 676, 651, 626, 601, 576, 551, 526, 501, 476, 450, 425, 400, 375, 350, 325, 300};
volatile char FlagOHigh=0,FlagOLow=0,FlagOOverCurrent=0,FlagBattLow=0,FlagFaultIndicator=0,FlagIHigh=0,FlagILow=0,FlagIOverCurrent=0,flag=0,softstart=1;
 
/*
Variable Declaration required for each PID controller in your application
*/
/* Declare a PID Data Structure named, fooPID */
tPID fooPID;
/* The fooPID data structure contains a pointer to derived coefficients in X-space and */
/* pointer to controler state (history) samples in Y-space. So declare variables for the */
/* derived coefficients and the controller history samples */
fractional abcCoefficient[3] __attribute__ ((section (".xbss, bss, xmemory")));
fractional controlHistory[3] __attribute__ ((section (".ybss, bss, ymemory")));
/* The abcCoefficients referenced by the fooPID data structure */
/* are derived from the gain coefficients, Kp, Ki and Kd */
/* So, declare Kp, Ki and Kd in an array */
fractional kCoeffs[] = {0,0,0};


//*********************************************************************************************************************************************************
//                                                   FUNCTION DECLARATIONS
//**********************************************************************************************************************************************************************
void InitializePWM1(void);
void InitializeADC(void);
//void InitializeInterrupt(void);

void TMR3Intialize(void);
void TMR5Intialize(void);

int GetAdcValue(char channel);
void CheckOutputCurrent(void);
void CheckOutputVoltage(void);
void CheckOutputVoltageHigh(void);
void CheckInputVoltage(void);
void CheckInputCurrent(void);
void CheckIpConditions(void);
void CheckFaultConditions(void);

void delay(unsigned int);
void Lcd_Chr_Cp(char out_char);
void Lcd_Chr(char row, char column, char out_char);
void Lcd_Out_Cp(char *text);
void Lcd_Out(char row, char column, char *text);
void Lcd_Cmd(char out_char);
void Lcd_Initialize(void);

void ConfigurePorts(void);

// Interrupt function
void __attribute__((interrupt, no_auto_psv)) _PWMInterrupt(void){
       volatile static int softstart=1,PIDSetpoint=0,prevtemp=0;
        i++;
        if(i==512)
        {
            i=0;
            LATBbits.LATB7=~LATBbits.LATB7;
            if(softstart==1){
                fooPID.controlReference=(++PIDSetpoint);
                if(PIDSetpoint==545){
                    softstart=0;
                    PIDSetpoint=PIDRefrenceVal;
                    fooPID.controlReference=PIDSetpoint;
                }
            }
        }
        if(i%120==0)
        {
//                if(softstart==0){
//                   temp=GetAdcValue(ANA0_PWM_Control);
//                    if(temp>(prevtemp+20)||(temp<(prevtemp-20)))
//                    {
//                        fooPID.controlReference=temp;
 //                       PIDSetpoint=temp;
//                        prevtemp=temp;
//                    }
//                }
            temp=GetAdcValue(ANA1_Op_HighLow);
            //if(temp>(prevtemp+10)||temp<(prevtemp-10)){
                fooPID.measuredOutput=temp;
                prevtemp=temp;
            //}
                PID(&fooPID);
                result=fooPID.controlOutput+PIDSetpoint;
            if(result>1024)
            {
                result=1024;
                fooPID.controlOutput=(1024-PIDSetpoint);
            }
        }
        _PWMIF=0;
            
     
        k=i+1; 
        if(k==512)
        {
            k=0;
        } 
        PDC1=((LookUpTable[k]*result)>>10);
        if(softstart==1){
            CheckOutputVoltageHigh();   
        }
        else{
            CheckOutputVoltage();
        }
        CheckOutputCurrent();
        CheckInputVoltage();
        CheckInputCurrent();
        CheckIpConditions();
       // CheckFaultConditions();

}

 void _ISR _T3Interrupt(void){
    static unsigned long int count=0;
    _T3IF=0;
    TMR3_Reload();
    if(count==200){
    Overall_Fault_Indicator=1;
//   PDC1=0;
//   PWM1_StopPwm();
    TMR3_StopTimer();
    count=0;
    }
    TMR3=0;
    count++;
 }
 void _ISR _T5Interrupt(void){
    _T5IF=0;
    TMR5_Reload();
    Charger_Card=1;
    TMR5_StopTimer(); 
 }


//*********************************************************************************************************************************************************
//                                                             MAIN CODE
// *********************************************************************************************************************************************************** 
int main(void) {
    ConfigurePorts();
    InitializePWM1();
    InitializeADC();
    TMR3Intialize();
    TMR5Intialize();
    Lcd_Initialize();
    Lcd_Out(2,2,"Dev");
/*
Step 1: Initialize the PID data structure, fooPID
*/
    fooPID.abcCoefficients = &abcCoefficient[0];    /*Set up pointer to derived coefficients */
    fooPID.controlHistory = &controlHistory[0];     /*Set up pointer to controller history samples */
    PIDInit(&fooPID);                               /*Clear the controler history and the controller output */
	kCoeffs[0] = Q15(0.3);
	kCoeffs[1] = Q15(0.07);
	kCoeffs[2] = Q15(0.09);
    PIDCoeffCalc(&kCoeffs[0], &fooPID);             /*Derive the a,b, & c coefficients from the Kp, Ki & Kd */
    fooPID.controlReference =0;           /*Set the Reference Input for your controller */
    PDC1=0;
    EnableTimer3Interrupt();
    EnableTimer5Interrupt();
    EnablePwmInterrupt();
    while(1){
        if(Overall_Fault_Indicator==1){
            CheckOutputVoltage();
            CheckOutputCurrent();
            CheckInputVoltage();
            CheckInputCurrent();
            CheckIpConditions();
        }
        
    }
    return -1;
}
//**********************************************************************************************************************************************************************************
//                                                          FUNCTION DEFINITIONS
//**********************************************************************************************************************************************************************************

void InitializePWM1(){
    INTCON1=0x0000;
    INTCON2=0x0000;
    IFS2bits.PWMIF=0;
    IEC2bits.PWMIE=0;
    PTCON=0x2000;
    PTPER=2303;
    PWMCON1=0x0F10;
    PWMCON2=0x0000;
    PDC1=112;
    PTCONbits.PTEN=1;
}
void InitializeADC(){
    ADPCFG=0xFF80;
    ADCON2bits.VCFG=0;
    ADCON3bits.ADCS=9;
    ADCON2bits.CHPS=0;
    ADCON1bits.SIMSAM=1;
    ADCSSL=0x0000;
    ADCHS=0x0000;
    ADCON1bits.SSRC=7;
    ADCON1bits.ASAM=0;
    ADCON1bits.DONE=0;
    ADCON1bits.ADON=0;
    ADCON1bits.SAMP=1;
    ADCON3bits.SAMC=1;
    ADCON1bits.FORM=0;
    ADCON2bits.BUFS=0;
    ADCON2bits.SMPI=0;
    ADCON2bits.BUFM=0;
    ADCON2bits.ALTS=0;
    ADC_ON();
}
void TMR3Intialize(void){
    T3CON=0;
    _T3IP=6;
    TMR3=0;
    T3CONbits.TCKPS0=1;
    T3CONbits.TCKPS1=1;
    PR3=1152;
    _T3IF=0;
    _T3IE=0;
    
}
void TMR5Intialize(void){
    T5CON=0;
    _T5IP=5;
    TMR5=0;
    T5CONbits.TCKPS0=1;
    T5CONbits.TCKPS1=1;
    PR3=1152;
    _T5IF=0;
    _T5IE=0;
    
}

void CheckOutputVoltage(void){
   volatile static unsigned int AnalogVal=0;
    AnalogVal=GetAdcValue(ANA1_Op_HighLow);
    if(AnalogVal>573){
        FlagOHigh=1;
        Op_High_Indicator=1;
    }
    else{
        FlagOHigh=0;
        Op_High_Indicator=0;
    }
        
    if(AnalogVal<490){
        FlagOLow=1;
        Op_Low_Indicator=1;
    }
    else{
        FlagOLow=0;
        Op_Low_Indicator=0;
    }
}
void CheckOutputVoltageHigh(void){
   volatile static unsigned int AnalogVal=0;
    AnalogVal=GetAdcValue(ANA1_Op_HighLow);
    if(AnalogVal>573){
        FlagOHigh=1;
        Op_High_Indicator=1;
    }
    else{
        FlagOHigh=0;
        Op_High_Indicator=0;
    } 
}
void CheckOutputCurrent(void){
    volatile static unsigned int AnalogVal=0;
    AnalogVal=GetAdcValue(ANA2_Op_Overcurrent);
    if(AnalogVal>512)   {       
        FlagOOverCurrent=1;
        Op_Overcurrent_Indicator=1;
    }
    else {
        FlagOOverCurrent=0;
        Op_Overcurrent_Indicator=0;
    }
}
void CheckInputVoltage(void){
    volatile static unsigned int AnalogVal=0;
    AnalogVal=GetAdcValue(ANA3_Ip_Voltage);
    if(AnalogVal>573){
        FlagIHigh=1;
        Mains_High_Indicator=1;
    }
    else{
        Mains_High_Indicator=0;
        FlagIHigh=0;
    }
    
    if(AnalogVal<490){
        FlagILow=1;
        Main_Low_Indicator=1;
    }
    else{
        FlagILow=0;
        Main_Low_Indicator=0;
    }
}
void CheckInputCurrent(void){
    volatile static unsigned int AnalogVal=0;
    AnalogVal=GetAdcValue(ANA4_Ip_Current);
    if(AnalogVal>512)   {       
        FlagIOverCurrent=1;
    }
    else {
        FlagIOverCurrent=0;
    }
}
void CheckIpConditions(void){
    if(FlagIHigh==1||FlagILow==1||FlagIOverCurrent==1){
        TMR5_StartTimer();
    }
    else{
        Charger_Card=0;
        TMR5_StopTimer();
        TMR5_Reload();
    }
}
void CheckFaultConditions(void){
    if(FlagOHigh||FlagOLow||FlagOOverCurrent){
        TMR3_StartTimer();
    }
    else {
        TMR3_StopTimer();
        TMR3_Reload();
        Overall_Fault_Indicator=0;
        
    }
}

void delay(unsigned int j){
                unsigned int i=0,k=0;
                for(k=0;k<10;k++){
                for(i=0;i<=j;i++);
                }
                
}
void Lcd_Chr_Cp(char out_char){
    char Temp=0;
    LCD_RS=1;
    Temp=out_char;
    Temp&=0xF0;
    Temp=Temp>>2;
    LATE&=0xC3;
    LATE|=Temp;
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
    
    Temp=out_char;
    Temp&=0x0F;
    Temp=Temp<<2;
    LATE&=0xC3;
    LATE|=Temp;
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
}
void Lcd_Chr(char row, char column, char out_char){
    char CommVal=0;
    if(row==1){
        CommVal=0x80;
    }
    else{
        CommVal=0xC0;
    }
    CommVal=CommVal+column;
    Lcd_Cmd(CommVal);
    Lcd_Chr_Cp(out_char);
}
void Lcd_Out_Cp(char *text){
    unsigned int i=0;
    for(;text[i]!=0;i++)
    Lcd_Chr_Cp(text[i]);
}
void Lcd_Out(char row, char column, char *text){
    unsigned int i=0;
    char CommVal=0;
    if(row==1){
        CommVal=0x80;
    }
    else{
        CommVal=0xC0;
    }
    CommVal=CommVal+column;
    Lcd_Cmd(CommVal);
    for(;text[i]!=0;i++)
    Lcd_Chr_Cp(text[i]);
    
    
}
void Lcd_Cmd(char out_char){
    char Temp=0;
    LCD_RS=0;
    Temp=out_char;
    Temp&=0xF0;
    Temp=Temp>>2;
    LATE&=0xC3;
    LATE|=Temp;
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
    
    Temp=out_char;
    Temp&=0x0F;
    Temp=Temp<<2;
    LATE&=0xC3;
    LATE|=Temp;
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
 
}
void Lcd_Initialize(void){
    delay(Lcd_delay);
   // lcd_init_write(0x30);   //Special Sequence:Write Function Set.
    LATE|=0x0C;
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
    delay(Lcd_delay);
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
    delay(Lcd_delay);
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
    delay(Lcd_delay);
    Lcd_Cmd(0x08);                                                // Clear Screen & Returns the Cursor Home
    delay(Lcd_delay);
    Lcd_Cmd(0x06);      
    delay(Lcd_delay);      
    Lcd_Cmd(0x0C);           //Inc cursor to the right when writing and don?t shift screen
    delay(Lcd_delay);
    Lcd_Cmd(0x01);
    delay(Lcd_delay);
    Lcd_Cmd(0x80);
    delay(Lcd_delay);
    Lcd_Cmd(0xC0);
    delay(Lcd_delay);
}
void ConfigurePorts(){
    TRISB=0xFFF;            //Setting as input for analog function
    TRISBbits.TRISB7=0; //Setting as o/p
    TRISBbits.TRISB8=0;
    TRISE=0;
    TRISF=0;
    TRISD=0;
    TRISC=0;
    LATB=0;
    LATE=0;
    LATF=0;
    LATC=0;
    LATD=0;
    LATBbits.LATB7=1;
}

int GetAdcValue(char channel){
    ADCHS=channel;
    ADC_StartSampling();
    while(!ADCON1bits.DONE);
    return ADCBUF0;
}
//***************************************************************************************************************************************************************
//                                               END OF CODE
//***************************************************************************************************************************************************************