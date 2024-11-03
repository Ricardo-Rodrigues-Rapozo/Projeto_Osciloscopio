#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"  // Necessário para configurar os pinos UART
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "inc/hw_types.h"
#include "driverlib/timer.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/fpu.h"


//
//--------------------------- PARAMETROS ---------------------------------------------------------------------
//

#define PWM_FREQUENCY 100
#define APP_PI 3.1415926535897932384626433832795f
#define STEPS 256
uint32_t ui32StoredPWM;
#define SIZE_BUFFER 5000
uint16_t  buffer[SIZE_BUFFER];
volatile uint8_t flag = 0;
uint16_t cont = 0;
uint32_t FS = 40000; // Frequência de amostragem
uint8_t SYNC_BYTE = 1;  // Sinal de sincronização esperado
uint32_t ui32ADC0Value[1]; // Armazena valores do ADC
uint16_t adcValue16;
uint32_t control1 = 0; // variavel para debug
uint32_t control2 = 0; // variavel para debug
uint32_t ui32SysClkFreq;
uint32_t timerLoad;
volatile uint32_t ui32Load; // PWM period
volatile uint32_t ui32BlueLevel; // PWM duty cycle for blue LED
volatile uint32_t ui32PWMClock; // PWM clock frequency
volatile uint32_t ui32Index; // Counts the calculation loops
float fAngle; // Value for sine math (radians)


//
// ---------------------Configuração PWM ----------------------------------------------------------------------
//
void InitPWM(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG); // Change these pins later
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Change these pins later
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Check where this PWM has it's generation on

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_3); // Change these pins later
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_3, 0x00); // Change these pins later

    PWMClockSet(PWM0_BASE,PWM_SYSCLK_DIV_8 ); //Setting frenquency

    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);

    ui32PWMClock = ui32SysClkFreq / 64; // 120MHz/64
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1; // 1875000/100

    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ui32Load);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ui32Load/2);
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

    ui32Index = 0;

}

//
//--------------------Interrupção para RX UART ----------------------------------------------------------------
//

void UARTIntHandler(void)
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status
    UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts
    control1 = 10;
    while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
    {
        if(UARTCharGetNonBlocking(UART0_BASE) == SYNC_BYTE)
        {
            flag = 1;
        }
    }
}

//
//-------------------------- Inicialização do UART --------------------------------------------------------------------------
//
void UARTInit(void) {
//
//--------------------Enable the UART peripheral ---------------------------------------------------------------------------------------------
//
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//
//--------------------Set the Rx/Tx pins as UART pins ----------------------------------------------------------------
//
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//
//-------------------- Configure the UART baud rate, data configuration ----------------------------------------------------------------
//
    UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 115200,(UART_CONFIG_WLEN_8 |
                        UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

//
//-------------------- Configure the Interrupt ----------------------------------------------------------------
//
    IntMasterEnable();
    IntEnable(INT_UART0);
    UARTIntEnable( UART0_BASE , UART_INT_RX | UART_INT_RT);

//
//-------------------- Enable UART ----------------------------------------------------------------
//
    //UARTEnable(UART0_BASE);
}

//
//------------------------- Função para enviar dados via UART -------------------------------------------
//
void sendINT32(uint16_t* buffer) {// Função para enviar 4 bytes via UART
    uint_fast16_t i;
    char* ptr = (char*)buffer;
    for ( i = 0; i < SIZE_BUFFER*sizeof(uint16_t) ; i++)
    {
        UARTCharPut(UART0_BASE, ptr[i]);  // Envia cada byte
    }
}



//
//---------------------------------- INICIALIZAÇAO DO ADC ----------------------------------------------------------
//
void InitADC(void)
    {
//
//---------------- Habilita o ADC, GPIO E e o Timer0 -----------------------------------------------------------
//
           SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // habilita o ADC0
           SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //  habilita a porta E GPIO
           SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // habilita o timer 0
//
//---------- Aguarda até que os periféricos estejam prontos ----------------------------------------------------------------------
//
           while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
           while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
           while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

//
//--------- Configura o pino PE3 como entrada analógica (AIN0)---------------------------------------
//
           GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

//
//----------Configura o Timer0 para gerar um trigger periódico para o ADC ----------------------------------------
//

           TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
           timerLoad = (SysCtlClockGet() / FS) - 1; // Configura para uma taxa de FS kHz
           TimerLoadSet(TIMER0_BASE, TIMER_A, timerLoad);
           TimerControlTrigger(TIMER0_BASE, TIMER_A, true); // Permite que o Timer dispare o ADC
           TimerEnable(TIMER0_BASE, TIMER_A);

//
// -------------------------- Configurações do ADC -------------------------------------------------------
// Configura o Sequenciador 1 para usar o Timer0 como trigger
           ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0);

//
//-------- Configura as etapas do sequenciador para ler o sinal de AIN0 (PE3) -----------------------------------
//
           ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

//
//--------------Habilita o Sequenciador 1 e limpa qualquer interrupção pendente --------------------------------------
//

           ADCSequenceEnable(ADC0_BASE, 1);
           ADCIntClear(ADC0_BASE, 1);
    }


int main(void)
{
    int idx=0;
//
// -------------------------- Configurações de clock e periféricos -------------------------------------------------------
// Configura o clock do sistema para 120 MHz

    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
//
// -------------------------- Inicialização da UART  ----------------------------------------------------------------------
//

    UARTInit();

//
// -------------------------- Inicialização do ADC ----------------------------------------------------------------------
//
    InitADC();

//
// -------------------------- Inicialização do PWM ----------------------------------------------------------------------
//

    InitPWM();
    ui32StoredPWM = ui32Load/2; // 1% de ciclo de trabalho
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ui32StoredPWM);
// -------------------------- Loop Principal ------------------------------------------------------------------
//

    while(1)
    {

        //SysCtlDelay(ui32SysClkFreq/(STEPS));
        // Espera até a conversão do ADC estar completa
        if (ADCIntStatus(ADC0_BASE, 1, false))
        {
            //Limpa a flag de interrupção do ADC e obtém os dados do sequenciador
            ADCIntClear(ADC0_BASE, 1);
            ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
            control2 = 2;
            buffer[idx] = (uint16_t) (ui32ADC0Value[0] & 0xFFFF);  // Mantém apenas os 16 bits menos significativos
            idx = (idx+1)%SIZE_BUFFER;

            if (flag == 1)
            {
                control2 = 10;
                sendINT32(buffer);
                flag = 0;
            }
        }
    }
}
