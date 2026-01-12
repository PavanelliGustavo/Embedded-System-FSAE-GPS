#include <msp430.h>
#include <string.h>      

// --- Variáveis Globais ---
#define BUFFER_SIZE 100
char rxBuffer[BUFFER_SIZE];      // Buffer da ISR (volátil)
char processBuffer[BUFFER_SIZE]; // Buffer de processamento do Main
int bufferIndex = 0;
volatile char newDataReady = 0; 

// Função para enviar uma string pela UART para o PC
void uart_puts(char *s) {
    while(*s) {
        while (!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = *s++;
    }
}

void main(void)
{
    // Parar o Watchdog Timer
    WDTCTL = WDTPW | WDTHOLD;

    // --- Configuração da UART 0 (GPS @ 38400) ---
    P3SEL |= BIT3 | BIT4; 
    UCA0CTL1 |= UCSWRST;
    UCA0CTL1 |= UCSSEL_2; // SMCLK
    UCA0BR0 = 27;
    UCA0BR1 = 0;
    UCA0MCTL = UCBRS_2; 
    UCA0CTL1 &= ~UCSWRST;
    UCA0IE |= UCRXIE; 

    // --- Configuração da UART 1 (PC @ 38400) ---
    P4SEL |= BIT4 | BIT5; 
    UCA1CTL1 |= UCSWRST;
    UCA1CTL1 |= UCSSEL_2; // SMCLK
    UCA1BR0 = 27;
    UCA1BR1 = 0;
    UCA1MCTL = UCBRS_2; 
    UCA1CTL1 &= ~UCSWRST;

    // Habilita interrupções globalmente
    __bis_SR_register(GIE); // GIE (General Interrupt Enable)

    // --- Loop Principal ---
    while(1)
    {
        // Se a ISR avisou que uma sentença nova chegou
        if (newDataReady)
        {
            // --- Seção Crítica ---
            // Desliga interrupções para copiar os dados com segurança
            __disable_interrupt(); 
            strcpy(processBuffer, rxBuffer); // Copia os dados
            newDataReady = 0;                // Reseta a flag
            __enable_interrupt();            // Liga as interrupções
            // --- Fim da Seção Crítica ---

            // Agora processamos a cópia (processBuffer) com segurança
            if (strncmp(processBuffer, "$GNGGA", 6) == 0)
            {
                char *token;
                char *latitude;
                char *ns; 
                char *longitude;
                char *ew; 
                char *fixQuality;
                int i = 0;

                // "Quebra" a CÓPIA, não o buffer original
                token = strtok(processBuffer, ","); 

                while (token != NULL)
                {
                    switch(i)
                    {
                        case 2: latitude = token; break;
                        case 3: ns = token; break;
                        case 4: longitude = token; break;
                        case 5: ew = token; break;
                        case 6: fixQuality = token; break;
                    }
                    token = strtok(NULL, ",");
                    i++;
                }

                // Se a qualidade do fix for '1' (GPS) ou '2' (DGPS)
                // (Verifica se 'fixQuality' não é nulo antes de ler)
                if (fixQuality != NULL && (*fixQuality == '1' || *fixQuality == '2'))
                {
                    uart_puts("Lat: ");
                    uart_puts(latitude);
                    uart_puts(ns);
                    uart_puts(" | Lon: ");
                    uart_puts(longitude);
                    uart_puts(ew);
                    uart_puts("\r\n"); // Nova linha
                }
            }
        }

        // Coloca o processador em modo de baixo consumo
        // A interrupção (case 2) irá acordá-lo
        __bis_SR_register(LPM0_bits + GIE);
        __no_operation();
    }
}

// --- Rotina de Interrupção da UART 0 (GPS) ---
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV, 4))
    {
        case 2: // RXIFG - Recebeu caractere do GPS
        {
            char c = UCA0RXBUF; 

            if (c == '\n') 
            {
                rxBuffer[bufferIndex] = '\0'; 
                bufferIndex = 0;              
                newDataReady = 1;             
                __bic_SR_register_on_exit(LPM0_bits); // Acorda o processador
            }
            else if (c != '\r' && bufferIndex < (BUFFER_SIZE - 1))
            {
                rxBuffer[bufferIndex++] = c; // Adiciona ao buffer
            }
            break;
        }
        default: break;
    }
}
