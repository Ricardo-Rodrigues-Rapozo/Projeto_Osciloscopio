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

//
//--------------------------- PARAMETROS ---------------------------------------------------------------------
//

#define SIZE_BUFFER 1000
int buffer[SIZE_BUFFER];
volatile uint8_t flag = 0;
volatile uint32_t ui32ADCAvg; // Média das leituras do ADC
uint16_t cont = 0;
uint32_t FS = 5000; // Frequência de amostragem
uint8_t SYNC_BYTE = 1;  // Sinal de sincronização esperado
uint32_t ui32ADC0Value[1]; // Armazena valores do ADC
uint32_t control1 = 0; // variavel para debug
uint32_t control2 = 0; // variavel para debug
uint32_t ui32SysClkFreq;
uint16_t i;

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
void sendINT32(int* buffer) {// Função para enviar 4 bytes via UART
    uint_fast16_t i;
    char* ptr = (char*)buffer;
    for ( i = 0; i < SIZE_BUFFER*sizeof(int) ; i++)
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
           uint32_t timerLoad = (SysCtlClockGet() / FS) - 1; // Configura para uma taxa de FS kHz
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
// -------------------------- BUFFER ----------------------------------------------------------------------
//

// -------------------------- Loop Principal ------------------------------------------------------------------
//

    while(1)
    {
        // Espera até a conversão do ADC estar completa
        if (ADCIntStatus(ADC0_BASE, 1, false))
        {
            // Limpa a flag de interrupção do ADC e obtém os dados do sequenciador
            ADCIntClear(ADC0_BASE, 1);
            ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
            control2 = 2;
            for(i =0; i < SIZE_BUFFER; i++)
            {
                buffer[i] = ui32ADC0Value[0];
            }
            if (flag == 1)
            {
                sendINT32(buffer);
            }

        }
    }
}
