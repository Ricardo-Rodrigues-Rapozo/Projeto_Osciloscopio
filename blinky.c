#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "inc/hw_uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/fpu.h"


#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif



#define PWM_FREQUENCY 100
#define APP_PI 3.1415926535897932384626433832795f
#define STEPS 256
uint32_t ui32StoredPWM;
uint32_t timerLoad;
volatile uint32_t ui32Load; // PWM period
volatile uint32_t ui32BlueLevel; // PWM duty cycle for blue LED
volatile uint32_t ui32PWMClock; // PWM clock frequency
volatile uint32_t ui32Index; // Counts the calculation loops
float fAngle; // Value for sine math (radians)
// Tamanho do vetor e declaração do vetor de 16 bits
#define VECTOR_SIZE 200
uint16_t dataVector[VECTOR_SIZE];
uint32_t ui32SysClkFreq;
uint32_t FS = 40000;
uint8_t SYNC_BYTE = 1;  // Sinal de sincronização esperado
uint32_t ui32ADC0Value[1]; // Armazena valores do ADC
volatile uint8_t flag = 0;
int idx = 0;
uint16_t control1 = 0;


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


void UARTIntHandler(void)
{
    uint32_t ui32Status;
    control1 = 10;
    ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status
    UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts
    while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
    {
        if(UARTCharGetNonBlocking(UART0_BASE) == SYNC_BYTE)
        {
            flag = 1;
        }
    }
}

// Função de inicialização do UART
void UART_Init(void) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

        GPIOPinConfigure(GPIO_PA0_U0RX);
        GPIOPinConfigure(GPIO_PA1_U0TX);
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

        UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

        UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

        UARTEnable(UART0_BASE);

        UARTDMAEnable(UART0_BASE, UART_DMA_TX);
        // ------------------ Habilitar interrupções UART ------------------

            // Registra a função de interrupção UARTIntHandler como a função que será chamada quando uma interrupção ocorrer
            UARTIntRegister(UART0_BASE, UARTIntHandler);

            // Habilitar as interrupções para eventos de recepção (RX) e timeout de recepção (RT)
            UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

            // Habilitar a interrupção UART0 no NVIC (controlador de interrupções do microcontrolador)
            IntEnable(INT_UART0);

            // Habilitar interrupções globais no sistema (caso ainda não estejam habilitadas)
            IntMasterEnable();
}

// Função de inicialização do uDMA
void uDMA_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(pui8ControlTable);
}

// Configura o canal uDMA para transferência de dados
void configure_uDMAChannel(void) {
    uDMAChannelAssign(UDMA_CHANNEL_UART0TX);
    uDMAChannelAttributeDisable(UDMA_CHANNEL_UART0TX,
                                        UDMA_ATTR_ALTSELECT |
                                        UDMA_ATTR_HIGH_PRIORITY |
                                        UDMA_ATTR_REQMASK);

    uDMAChannelAttributeEnable(UDMA_CHANNEL_UART0TX, UDMA_ATTR_USEBURST);

    uDMAChannelControlSet(UDMA_CHANNEL_UART0TX | UDMA_PRI_SELECT,
                          UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_8);

    uDMAChannelTransferSet(UDMA_CHANNEL_UART0TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
                           dataVector, (void *)(UART0_BASE + UART_O_DR), VECTOR_SIZE*sizeof(uint32_t));


}


void ConfigureADC(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32SysClkFreq/FS - 1);

    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

    TimerEnable(TIMER0_BASE, TIMER_A);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 1);

    ADCIntClear(ADC0_BASE, 1);


}

void Send_DMA(void)
{

    uDMAChannelTransferSet(UDMA_CHANNEL_UART0TX | UDMA_PRI_SELECT,
                           UDMA_MODE_BASIC, dataVector,
                           (void *)(UART0_BASE + UART_O_DR), VECTOR_SIZE * sizeof(uint32_t));

    uDMAChannelEnable(UDMA_CHANNEL_UART0TX);
    control1 = 20;

}

// Função principal
int main(void) {
    // Configuração do clock do sistema
    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
        SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // Inicialização do UART e uDMA
    ConfigureADC();
    UART_Init();
    uDMA_Init();
    configure_uDMAChannel();


    InitPWM();
    ui32StoredPWM = ui32Load/2; // 1% de ciclo de trabalho
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ui32StoredPWM);

    // Ativa a transferência uDMA
    uDMAChannelEnable(UDMA_CHANNEL_UART0TX);



    // Loop infinito para manter o programa em execução
    while (1) {
        // Verificar e reiniciar a transferência se necessário

        // Espera até a conversão do ADC estar completa
              if (ADCIntStatus(ADC0_BASE, 1, false))
              {
                  //Limpa a flag de interrupção do ADC e obtém os dados do sequenciador
                  ADCIntClear(ADC0_BASE, 1);
                  ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
                  dataVector[idx] = (uint16_t) ui32ADC0Value[0];  // Mantém apenas os 16 bits menos significativos
                  idx = (idx+1)%VECTOR_SIZE;


                  if (flag == 1)
                  {
                      Send_DMA();
                      flag = 0;
                  }
              }
        if (!uDMAChannelIsEnabled(UDMA_CHANNEL_UART0TX)) {
            uDMAChannelTransferSet(UDMA_CHANNEL_UART0TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
                                       dataVector, (void *)(UART0_BASE + UART_O_DR), VECTOR_SIZE*sizeof(uint32_t));
            uDMAChannelEnable(UDMA_CHANNEL_UART0TX);

        }
    }
}
