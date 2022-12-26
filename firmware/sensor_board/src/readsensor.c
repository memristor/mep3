#include "definitions.h"

#define S2 LATBbits.LATB14  
#define S3 LATCbits.LATC9 
#define S0 LATBbits.LATB12 //ne koristi se
#define S1 LATAbits.LATA10 //ne koristi se

int brojac1 = 0;
int brojac2 = 0;
int brojac5 = 0;
int brojac7 = 0;

int readSensor1();
int readSensor2();
int readSensor5();
int readSensor7();

int readSensor(int sensorNum) 
{
    int sensorValue = 0;
    
    switch(sensorNum)
    {        
            case 1: //sensor 1
                sensorValue = readSensor1();
                break;
            case 2:
                sensorValue = readSensor2();
                break;
            case 5:
                sensorValue = readSensor5();
                break;
            case 7:
                sensorValue = readSensor7();
                break;
            default:
                break;
    }
    
    return sensorValue;
}

int readSensor1()
{  
    
    uint16_t boja = 0 ;
    S2 = 0 ;
    S3 = 0 ;
    TMR1_Start();
        switch (brojac1)
                {
                    case 0: //red
                        TMR1_Stop();
                        boja = TMR1_CounterGet();
                        TMR1=0;
                        S2 = 0;
                        S3 = 0;
                        brojac1++;
                        TMR1_Start();
                        UART3_Write("R ", strlen("R "));
                        break;
                    case 1: //clear
                        TMR1_Stop();
                        boja = TMR1_CounterGet();
                        TMR1=0;
                        S2 = 1;
                        S3 = 0;
                        brojac1++;
                        TMR1_Start();
                        UART3_Write("W ", strlen("W "));
                        break;
                    case 2: //blue
                        TMR1_Stop();
                        boja = TMR1_CounterGet();
                        TMR1=0;
                        S2 = 0;
                        S3 = 1;
                        brojac1++;
                        TMR1_Start();
                        UART3_Write("B ", strlen("B "));
                        break;
                    case 3: //green
                        TMR1_Stop();
                        boja = TMR1_CounterGet();
                        TMR1=0;
                        S2 = 1;
                        S3 = 1;
                        brojac1 = 0;
                        TMR1_Start();
                        UART3_Write("Z ", strlen("Z "));
                        break;
                    default:
                        break;
                }
        return boja;
}

int readSensor2()
{  
    
    uint16_t boja = 0 ;
    S2 = 0 ;
    S3 = 0 ;
    TMR2_Start();
        switch (brojac2)
                {
                    case 0: //red
                        TMR2_Stop();
                        boja = TMR2_CounterGet();
                        TMR2=0;
                        S2 = 0;
                        S3 = 0;
                        brojac2++;
                        TMR2_Start();
                        UART3_Write("R ", strlen("R "));
                        break;
                    case 1: //clear
                        TMR2_Stop();
                        boja = TMR2_CounterGet();
                        TMR2=0;
                        S2 = 1;
                        S3 = 0;
                        brojac2++;
                        TMR2_Start();
                        UART3_Write("W ", strlen("W "));
                        break;
                    case 2: //blue
                        TMR2_Stop();
                        boja = TMR2_CounterGet();
                        TMR2=0;
                        S2 = 0;
                        S3 = 1;
                        brojac2++;
                        TMR2_Start();
                        UART3_Write("B ", strlen("B "));
                        break;
                    case 3: //green
                        TMR2_Stop();
                        boja = TMR2_CounterGet();
                        TMR2=0;
                        S2 = 1;
                        S3 = 1;
                        brojac2 = 0;
                        TMR2_Start();
                        UART3_Write("Z ", strlen("Z "));
                        break;
                    default:
                        break;
                }
        return boja;
}

int readSensor5()
{  
    
    uint16_t boja = 0 ;
    S2 = 0 ;
    S3 = 0 ;
    TMR5_Start();
        switch (brojac5)
                {
                    case 0: //red
                        TMR5_Stop();
                        boja = TMR5_CounterGet();
                        TMR5=0;
                        S2 = 0;
                        S3 = 0;
                        brojac5++;
                        TMR5_Start();
                        UART3_Write("R ", strlen("R "));
                        break;
                    case 1: //clear
                        TMR5_Stop();
                        boja = TMR5_CounterGet();
                        TMR5=0;
                        S2 = 1;
                        S3 = 0;
                        brojac5++;
                        TMR5_Start();
                        UART3_Write("W ", strlen("W "));
                        break;
                    case 2: //blue
                        TMR5_Stop();
                        boja = TMR5_CounterGet();
                        TMR5=0;
                        S2 = 0;
                        S3 = 1;
                        brojac5++;
                        TMR5_Start();
                        UART3_Write("B ", strlen("B "));
                        break;
                    case 3: //green
                        TMR5_Stop();
                        boja = TMR5_CounterGet();
                        TMR5=0;
                        S2 = 1;
                        S3 = 1;
                        brojac5 = 0;
                        TMR5_Start();
                        UART3_Write("Z ", strlen("Z "));
                        break;
                    default:
                        break;
                }
        return boja;
}

int readSensor7()
{  
    
    uint16_t boja = 0 ;
    S2 = 0 ;
    S3 = 0 ;
    TMR7_Start();
        switch (brojac7)
                {
                    case 0: //red
                        TMR7_Stop();
                        boja = TMR7_CounterGet();
                        TMR7=0;
                        S2 = 0;
                        S3 = 0;
                        brojac7++;
                        TMR7_Start();
                        UART3_Write("R ", strlen("R "));
                        break;
                    case 1: //clear
                        TMR7_Stop();
                        boja = TMR7_CounterGet();
                        TMR7=0;
                        S2 = 1;
                        S3 = 0;
                        brojac7++;
                        TMR7_Start();
                        UART3_Write("W ", strlen("W "));
                        break;
                    case 2: //blue
                        TMR7_Stop();
                        boja = TMR7_CounterGet();
                        TMR7=0;
                        S2 = 0;
                        S3 = 1;
                        brojac7++;
                        TMR7_Start();
                        UART3_Write("B ", strlen("B "));
                        break;
                    case 3: //green
                        TMR7_Stop();
                        boja = TMR7_CounterGet();
                        TMR7=0;
                        S2 = 1;
                        S3 = 1;
                        brojac7 = 0;
                        TMR7_Start();
                        UART3_Write("Z ", strlen("Z "));
                        break;
                    default:
                        break;
                }
        return boja;
}