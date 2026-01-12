#include <msp430.h>

/*
 * Código "Ponte" (Bridge) GPS -> PC
 * Ambas as portas configuradas para 38400 bps (configuração de fábrica do GPS)
 */

void main(void)
{
    // Parar o Watchdog Timer
    WDTCTL = WDTPW | WDTHOLD;

    // --- Configuração da UART 0 (Conectada ao GPS @ 38400) ---
    P3SEL |= BIT3 | BIT4; // P3.3 = TX, P3.4 = RX
    UCA0CTL1 |= UCSWRST;
    UCA0CTL1 |= UCSSEL_2; // SMCLK
    
    // Configuração para 38400 bps (Assumindo clock de ~1MHz)
    // 1048576 / 38400 = 27.306
    // UCBRx = 27
    // UCBRSx = 2 (para a fração 0.306)
    UCA0BR0 = 27;
    UCA0BR1 = 0;
    UCA0MCTL = UCBRS_2; // Ajuste para 38400
    
    UCA0CTL1 &= ~UCSWRST;
    UCA0IE |= UCRXIE; // Habilita interrupção de recebimento

    // --- Configuração da UART 1 (Conectada ao PC @ 38400) ---
    P4SEL |= BIT4 | BIT5; // P4.4 = TX, P4.5 = RX
    UCA1CTL1 |= UCSWRST;
    UCA1CTL1 |= UCSSEL_2; // SMCLK
    
    // Configuração para 38400 bps
    UCA1BR0 = 27;
    UCA1BR1 = 0;
    UCA1MCTL = UCBRS_2; // Ajuste para 38400
    
    UCA1CTL1 &= ~UCSWRST;

    // Habilita interrupções e entra em modo de baixo consumo
    __bis_SR_register(LPM0_bits + GIE);
    __no_operation();
}

// --- Rotina de Interrupção da UART 0 (GPS) ---
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV, 4))
    {
        case 0: break;
        case 2: // RXIFG - Recebeu caractere do GPS
        {
            char charDoGPS = UCA0RXBUF;
            while (!(UCA1IFG & UCTXIFG)); // Espera PC estar pronto
            UCA1TXBUF = charDoGPS;        // Envia para o PC
            break;
        }
        case 4: break;
        default: break;
    }
}
