#include <msp430.h>
#include <string.h>
#include <stdlib.h>

/*
 * MSP430 GPS Datalogger
 * Envia "LATITUDE,LONGITUDE" pela Serial continuamente.
 */

#define BUFFER_SIZE 100
char rxBuffer[BUFFER_SIZE];
char processBuffer[BUFFER_SIZE];
int bufferIndex = 0;
volatile char newDataReady = 0;

void uart_puts(char *s) {
    while(*s) {
        while (!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = *s++;
    }
}

// Função auxiliar para converter NMEA para Decimal
// Transforma 2333.038S em -23.5506
void enviarDecimal(char *nmeaCoord, char direcao) {
    double raw = atof(nmeaCoord);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);
    if (direcao == 'S' || direcao == 'W') decimal *= -1;

    // Imprimir float manualmente
    if (decimal < 0) { uart_puts("-"); decimal = -decimal; }
    int intPart = (int)decimal;
    long fracPart = (long)((decimal - intPart) * 1000000); // 6 casas decimais
    
    char buf[20];
    ltoa(intPart, buf, 10);
    uart_puts(buf);
    uart_puts(".");
    ltoa(fracPart, buf, 10);
    uart_puts(buf);
}

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    // Configurações de UART (38400 bps)
    P3SEL |= BIT3 | BIT4; // GPS
    UCA0CTL1 |= UCSWRST; UCA0CTL1 |= UCSSEL_2;
    UCA0BR0 = 27; UCA0BR1 = 0; UCA0MCTL = UCBRS_2;
    UCA0CTL1 &= ~UCSWRST; UCA0IE |= UCRXIE;

    P4SEL |= BIT4 | BIT5; // PC
    UCA1CTL1 |= UCSWRST; UCA1CTL1 |= UCSSEL_2;
    UCA1BR0 = 27; UCA1BR1 = 0; UCA1MCTL = UCBRS_2;
    UCA1CTL1 &= ~UCSWRST;

    __bis_SR_register(GIE);

    while(1)
    {
        if (newDataReady)
        {
            __disable_interrupt();
            strcpy(processBuffer, rxBuffer);
            newDataReady = 0;
            __enable_interrupt();

            // Usando GNGGA ou GNRMC
            if (strncmp(processBuffer, "$GNGGA", 6) == 0)
            {
                char *token;
                char *lat, *ns, *lon, *ew, *fix;
                int i = 0;
                
                // Quebra a string
                token = strtok(processBuffer, ",");
                while (token != NULL) {
                    switch(i) {
                        case 2: lat = token; break;
                        case 3: ns = token; break;
                        case 4: lon = token; break;
                        case 5: ew = token; break;
                        case 6: fix = token; break;
                    }
                    token = strtok(NULL, ",");
                    i++;
                }

                // Se Fix válido (1 ou 2), envia para o PC
                if (fix != NULL && (*fix == '1' || *fix == '2')) {
                    // Formato CSV: LAT,LON
                    enviarDecimal(lat, *ns);
                    uart_puts(",");
                    enviarDecimal(lon, *ew);
                    uart_puts("\n"); // Nova linha = fim do pacote
                }
            }
        }
        __bis_SR_register(LPM0_bits + GIE);
        __no_operation();
    }
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV, 4)) {
        case 2: {
            char c = UCA0RXBUF;
            if (c == '\n') {
                rxBuffer[bufferIndex] = '\0';
                bufferIndex = 0;
                newDataReady = 1;
                __bic_SR_register_on_exit(LPM0_bits);
            } else if (c != '\r' && bufferIndex < BUFFER_SIZE - 1) {
                rxBuffer[bufferIndex++] = c;
            }
            break;
        }
    }
}
