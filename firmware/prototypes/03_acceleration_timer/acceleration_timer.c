#include <msp430.h>
#include <string.h>
#include <stdlib.h>
#include <math.h> // Necessário para Haversine

// --- Configurações e Constantes ---
#define BUFFER_SIZE 120
#define RAIO_TERRA 6371000.0 // Raio da Terra em metros
#define PI 3.14159265359
#define VELOCIDADE_TRIGGER 1.5 // Velocidade em km/h para considerar que "largou"

// --- Variáveis Globais ---
char rxBuffer[BUFFER_SIZE];
char processBuffer[BUFFER_SIZE];
int bufferIndex = 0;
volatile char newDataReady = 0;

// Variáveis de Estado
enum Estados { AGUARDANDO_FIX, PRONTO, CORRENDO, FINALIZADO };
int estadoAtual = AGUARDANDO_FIX;

// Variáveis de Dados
double startLat = 0.0, startLon = 0.0;
double currentLat = 0.0, currentLon = 0.0;
double currentSpeed = 0.0; // Em km/h
double distanciaPercorrida = 0.0;

// Variáveis de Tempo
volatile unsigned long millis = 0; // Contador de milissegundos
unsigned long tempoInicial = 0;
unsigned long tempoFinal = 0;

// --- Protótipos de Função ---
void uart_puts(char *s);
void uart_print_float(double val, int precision);
double converterNMEAparaDecimal(char *nmeaCoord, char direcao);
double calcularDistancia(double lat1, double lon1, double lat2, double lon2);
double toRadians(double deg);

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Para Watchdog

    // --- Configuração de Clock e Pinos ---
    // Configurar UART GPS (P3.3/P3.4) e PC (P4.4/P4.5) - Baud Rate 38400
    P3SEL |= BIT3 | BIT4;
    UCA0CTL1 |= UCSWRST; UCA0CTL1 |= UCSSEL_2;
    UCA0BR0 = 27; UCA0BR1 = 0; UCA0MCTL = UCBRS_2; // 38400 bps
    UCA0CTL1 &= ~UCSWRST; UCA0IE |= UCRXIE;

    P4SEL |= BIT4 | BIT5;
    UCA1CTL1 |= UCSWRST; UCA1CTL1 |= UCSSEL_2;
    UCA1BR0 = 27; UCA1BR1 = 0; UCA1MCTL = UCBRS_2; // 38400 bps
    UCA1CTL1 &= ~UCSWRST;

    // --- Configuração do Timer A0 para contar milissegundos ---
    // Assumindo SMCLK ~1MHz.
    TA0CCTL0 = CCIE; // Habilita interrupção do Timer A0
    TA0CCR0 = 1048;  // ~1ms (1048576 Hz / 1000)
    TA0CTL = TASSEL_2 + MC_1; // SMCLK, Up mode

    __bis_SR_register(GIE); // Habilita interrupções

    uart_puts("\r\n--- Cronometro GPS 75m Iniciado ---\r\n");
    uart_puts("Aguardando sinal GPS valido...\r\n");

    while(1)
    {
        if (newDataReady)
        {
            // --- Área Crítica: Cópia do Buffer ---
            __disable_interrupt();
            strcpy(processBuffer, rxBuffer);
            newDataReady = 0;
            __enable_interrupt();

            // Processa apenas sentenças $GNRMC (ou $GPRMC)
            // Formato: $GNRMC,Hora,Status,Lat,NS,Lon,EW,Vel,Curso,Data...
            if (strncmp(processBuffer, "$GNRMC", 6) == 0 || strncmp(processBuffer, "$GPRMC", 6) == 0)
            {
                char *p = processBuffer;
                char *tokens[12];
                int i = 0;

                // Parser simples (strtok destrói a string, ok pois é buffer cópia)
                tokens[i] = strtok(p, ",");
                while(tokens[i] != NULL && i < 12) {
                    tokens[++i] = strtok(NULL, ",");
                }

                // Verifica se tem campos suficientes e se Status é 'A' (Válido)
                // tokens[2] = Status
                if (i > 8 && *tokens[2] == 'A')
                {
                    // 1. Extrair Dados Brutos
                    double latRaw = atof(tokens[3]);
                    char latDir = *tokens[4];
                    double lonRaw = atof(tokens[5]);
                    char lonDir = *tokens[6];
                    double speedKnots = atof(tokens[7]);

                    // 2. Converter para unidades úteis
                    currentLat = converterNMEAparaDecimal(tokens[3], latDir);
                    currentLon = converterNMEAparaDecimal(tokens[5], lonDir);
                    currentSpeed = speedKnots * 1.852; // Nós -> Km/h

                    // --- LÓGICA DA MÁQUINA DE ESTADOS ---
                    switch (estadoAtual)
                    {
                        case AGUARDANDO_FIX:
                            uart_puts("GPS FIXO! Pronto para largar.\r\n");
                            estadoAtual = PRONTO;
                            break;

                        case PRONTO:
                            // Mostra velocidade atual para debug
                            // uart_puts("Vel: "); uart_print_float(currentSpeed, 1); uart_puts("\r\n");

                            // SE velocidade maior que trigger, LARGA!
                            if (currentSpeed > VELOCIDADE_TRIGGER)
                            {
                                startLat = currentLat;
                                startLon = currentLon;
                                tempoInicial = millis;
                                estadoAtual = CORRENDO;
                                uart_puts(">>> LARGADA DETECTADA! <<<\r\n");
                            }
                            break;

                        case CORRENDO:
                            // Calcular distância
                            distanciaPercorrida = calcularDistancia(startLat, startLon, currentLat, currentLon);

                            // Imprimir status
                            uart_puts("Dist: ");
                            uart_print_float(distanciaPercorrida, 2);
                            uart_puts("m | Tempo: ");
                            uart_print_float((millis - tempoInicial)/1000.0, 2);
                            uart_puts("s\r\n");

                            // Verifica Chegada
                            if (distanciaPercorrida >= 75.0)
                            {
                                tempoFinal = millis;
                                estadoAtual = FINALIZADO;
                                uart_puts("\r\n=== CHEGADA! ===\r\n");
                                uart_puts("Tempo Final: ");
                                uart_print_float((tempoFinal - tempoInicial)/1000.0, 3);
                                uart_puts(" segundos.\r\n");
                                uart_puts("Reinicie o microcontrolador para nova volta.\r\n");
                            }
                            break;

                        case FINALIZADO:
                            // Não faz nada, espera reset
                            break;
                    }
                }
                else
                {
                    if(estadoAtual == AGUARDANDO_FIX) uart_puts("."); // Loading...
                }
            }
        }
        __bis_SR_register(LPM0_bits + GIE); // Dorme
        __no_operation();
    }
}

// --- ISR do Timer A0 (Conta o Tempo) ---
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    millis++; // Incrementa a cada 1ms
}

// --- ISR da UART (Recebe GPS) ---
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV, 4))
    {
        case 2: // RX
        {
            char c = UCA0RXBUF;
            if (c == '\n') {
                rxBuffer[bufferIndex] = '\0';
                bufferIndex = 0;
                newDataReady = 1;
                __bic_SR_register_on_exit(LPM0_bits);
            }
            else if (c != '\r' && bufferIndex < BUFFER_SIZE - 1) {
                rxBuffer[bufferIndex++] = c;
            }
            break;
        }
    }
}

// --- Funções Auxiliares ---

// Converte NMEA (DDMM.MMMM) para Decimal Degrees (DD.DDDD)
// Ex: 2333.038, S -> -23.5506
double converterNMEAparaDecimal(char *nmeaCoord, char direcao)
{
    double raw = atof(nmeaCoord);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);
    if (direcao == 'S' || direcao == 'W') decimal *= -1;
    return decimal;
}

double toRadians(double deg) {
    return deg * (PI / 180.0);
}

// Fórmula de Haversine para distância em metros
double calcularDistancia(double lat1, double lon1, double lat2, double lon2)
{
    double dLat = toRadians(lat2 - lat1);
    double dLon = toRadians(lon2 - lon1);
    lat1 = toRadians(lat1);
    lat2 = toRadians(lat2);

    double a = sin(dLat/2) * sin(dLat/2) +
               sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return RAIO_TERRA * c;
}

void uart_puts(char *s) {
    while(*s) {
        while (!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = *s++;
    }
}

// Função simples para imprimir float (printf consome muita memória)
void uart_print_float(double val, int precision) {
    if (val < 0) { uart_puts("-"); val = -val; }
    int intPart = (int)val;
    long fracPart = (long)((val - intPart) * pow(10, precision));
    
    // Conversão manual para string
    char buf[20];
    ltoa(intPart, buf, 10); // Converte parte inteira
    uart_puts(buf);
    uart_puts(".");
    ltoa(fracPart, buf, 10); // Converte parte fracionária
    uart_puts(buf);
}
