// =============================================================================
// APUAMA RACING - SISTEMA DE GPS
// Firmware Unificado v1.0
// Autor: Gustavo Pavanelli
// Descrição: Firmware único para coleta de dados GPS nas provas dinâmicas
//            da Fórmula SAE. O modo de prova é definido pelo arquivo
//            /config.txt no cartão SD.
//
// Modos disponíveis:
//   "acceleration" → Cronômetro 0-75m, trigger por velocidade
//   "autocross"    → Traçado + tempo de volta única, trigger por geofence
//   "enduro"       → Traçado + contagem de voltas, trigger por geofence
//
// Operação em pista:
//   1. Posicionar o dispositivo NA LINHA DE LARGADA antes de ligar
//   2. Aguardar fix de qualidade (LED interno pisca lentamente)
//   3. O sistema captura automaticamente as coordenadas como ponto de largada
//   4. Aguardar 10 segundos (janela para afastar da linha)
//   5. O sistema inicia a lógica da prova selecionada
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <math.h>

// =============================================================================
// PINAGEM (ESP32 WROOM 38 PINOS)
// =============================================================================
#define RX_PIN_GPS   16   // UART2 RX ← TX do módulo GNSS
#define TX_PIN_GPS   17   // UART2 TX → RX do módulo GNSS
#define GPS_EN_PIN    4   // Habilita/desabilita o módulo GNSS
#define CS_PIN_SD     5   // Chip Select do módulo MicroSD

// =============================================================================
// PARÂMETROS DE CONFIGURAÇÃO
// Ajuste estes valores conforme necessário para calibração em pista
// =============================================================================

// --- Qualidade de Fix ---
#define MIN_SATELITES        6      // Mínimo de satélites para considerar fix válido
#define MAX_HDOP           2.5      // HDOP máximo aceitável (quanto menor, melhor)
#define TIMEOUT_FIX_MS   60000     // Tempo máximo aguardando fix (60 segundos)

// --- Geofence (Autocross / Enduro) ---
#define RAIO_GEOFENCE_M      10.0  // Raio em metros do ponto de largada
// O carro precisa SAI do geofence antes de poder cruzá-lo novamente.
// Isso evita falsos disparos enquanto o carro está parado na linha.
#define DIST_SAIDA_GEOFENCE  10.0  // Distância mínima da largada para considerar "saiu"

// --- Acceleration ---
#define VEL_INICIO_ACCEL_KMH  3.0  // Velocidade mínima para iniciar cronômetro (km/h)
#define DIST_PROVA_ACCEL_M   75.0  // Distância da prova de acceleration (metros)

// --- Janela de Largada ---
#define JANELA_LARGADA_MS   10000  // Tempo de espera após captura do ponto (ms)

// --- SD: Frequência de Flush ---
// A cada N linhas gravadas, o buffer é forçado para o cartão.
// Valor alto = menos escritas = menor risco de corrupção em caso de queda de energia
// Valor baixo = dados mais seguros porém maior latência
#define LINHAS_POR_FLUSH     20    // ~1 segundo a 20Hz com RMC+GGA ativos

// =============================================================================
// COMANDOS UBX (Protocolo U-Blox)
// Gerados com checksum correto para o chipset U-blox M10 (G10A-F30)
// =============================================================================

// Configura taxa de atualização para 20Hz (período de 50ms)
const byte UBX_CFG_RATE_20HZ[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
  0x32, 0x00, 0x01, 0x00, 0x01, 0x00,
  0x48, 0xE6
};

// Desliga mensagens desnecessárias (economia de banda obrigatória para 20Hz)
const byte UBX_DISABLE_GLL[] = {  // Lat/Lon redundante com RMC
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A
};
const byte UBX_DISABLE_GSA[] = {  // Satélites ativos - repetitivo
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31
};
const byte UBX_DISABLE_GSV[] = {  // Posição visual dos satélites - muito pesado
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38
};
const byte UBX_DISABLE_VTG[] = {  // Velocidade redundante com RMC
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46
};
const byte UBX_DISABLE_ZDA[] = {  // Data/hora redundante com RMC
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x08,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x4F
};

// =============================================================================
// ENUMERAÇÕES E ESTRUTURAS
// =============================================================================

enum ModoProva {
  MODO_DESCONHECIDO,
  MODO_ACCELERATION,
  MODO_AUTOCROSS,
  MODO_ENDURO
};

enum EstadoSistema {
  ESTADO_AGUARDANDO_FIX,       // Esperando GPS ter qualidade suficiente
  ESTADO_CAPTURANDO_LARGADA,   // Fix OK, capturando ponto de largada
  ESTADO_JANELA_LARGADA,       // Aguardando 10s para afastar da linha
  ESTADO_AGUARDANDO_INICIO,    // Esperando trigger de início da prova
  ESTADO_GRAVANDO,             // Prova em andamento, gravando dados
  ESTADO_FINALIZADO,           // Prova encerrada
  ESTADO_ERRO                  // Erro crítico (SD, config, etc.)
};

struct PontoGPS {
  double lat;
  double lon;
};

// =============================================================================
// VARIÁVEIS GLOBAIS
// =============================================================================

HardwareSerial SerialGPS(2);
TinyGPSPlus gps;
File logFile;

ModoProva    modoAtual        = MODO_DESCONHECIDO;
EstadoSistema estadoAtual     = ESTADO_AGUARDANDO_FIX;

String nomeArquivo            = "";
int    contadorFlush          = 0;

// Ponto de largada (capturado automaticamente após fix)
PontoGPS pontoLargada         = {0.0, 0.0};
bool     largadaCapturada     = false;
bool     saiuDoGeofence       = false;   // Flag: carro já saiu da área de largada?

// Cronômetro
unsigned long tempoInicio_ms  = 0;
unsigned long tempoFim_ms     = 0;

// Acceleration — ponto de início capturado no trigger para cálculo direto
// Usar distância direta (não integração incremental) evita acúmulo de ruído GPS (Random Walk)
PontoGPS pontoInicioAcel      = {0.0, 0.0};

// Enduro
int    numeroVolta            = 0;
unsigned long tempoVoltaInicio_ms = 0;

// Janela de largada
unsigned long tempoJanela_ms  = 0;

// =============================================================================
// FUNÇÕES AUXILIARES
// =============================================================================

// Calcula distância em metros entre dois pontos GPS usando Haversine
double calcularDistancia_m(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // Raio da Terra em metros
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// Envia comando UBX bruto para o módulo GNSS
void enviarComandoUBX(const byte *cmd, int tamanho) {
  for (int i = 0; i < tamanho; i++) SerialGPS.write(cmd[i]);
  SerialGPS.flush();
}

// Configura o módulo GNSS via protocolo UBX
void configurarGPS() {
  Serial.println("[GPS] Configurando U-blox M10 para 20Hz...");
  // Envia 3x para garantir recepção (GPS pode estar no meio de uma sentença)
  for (int i = 0; i < 3; i++) {
    enviarComandoUBX(UBX_DISABLE_GLL, sizeof(UBX_DISABLE_GLL)); delay(20);
    enviarComandoUBX(UBX_DISABLE_GSA, sizeof(UBX_DISABLE_GSA)); delay(20);
    enviarComandoUBX(UBX_DISABLE_GSV, sizeof(UBX_DISABLE_GSV)); delay(20);
    enviarComandoUBX(UBX_DISABLE_VTG, sizeof(UBX_DISABLE_VTG)); delay(20);
    enviarComandoUBX(UBX_DISABLE_ZDA, sizeof(UBX_DISABLE_ZDA)); delay(20);
    enviarComandoUBX(UBX_CFG_RATE_20HZ, sizeof(UBX_CFG_RATE_20HZ)); delay(100);
  }
  Serial.println("[GPS] Comandos UBX enviados.");
}

// Lê o modo de prova do arquivo /config.txt no SD
ModoProva lerModoDoSD() {
  if (!SD.exists("/config.txt")) {
    Serial.println("[CONFIG] ERRO: /config.txt não encontrado no SD!");
    return MODO_DESCONHECIDO;
  }

  File f = SD.open("/config.txt", FILE_READ);
  if (!f) {
    Serial.println("[CONFIG] ERRO: Não foi possível abrir /config.txt!");
    return MODO_DESCONHECIDO;
  }

  String conteudo = f.readStringUntil('\n');
  f.close();
  conteudo.trim();
  conteudo.toLowerCase();

  Serial.print("[CONFIG] Modo lido: '"); Serial.print(conteudo); Serial.println("'");

  if (conteudo == "acceleration") return MODO_ACCELERATION;
  if (conteudo == "autocross")    return MODO_AUTOCROSS;
  if (conteudo == "enduro")       return MODO_ENDURO;

  Serial.println("[CONFIG] ERRO: Modo desconhecido. Use: acceleration, autocross ou enduro");
  return MODO_DESCONHECIDO;
}

// Cria o arquivo CSV com nome baseado no modo e timestamp GPS
void criarArquivoCSV() {
  String prefixo;
  switch (modoAtual) {
    case MODO_ACCELERATION: prefixo = "ACCEL";  break;
    case MODO_AUTOCROSS:    prefixo = "AUTO";   break;
    case MODO_ENDURO:       prefixo = "ENDURO"; break;
    default:                prefixo = "GPS";    break;
  }

  // Usa timestamp do GPS se disponível, senão usa contador
  char filename[32];
  if (gps.date.isValid() && gps.time.isValid()) {
    sprintf(filename, "/%s_%02d%02d%02d_%02d%02d%02d.csv",
      prefixo.c_str(),
      gps.date.year() % 100, gps.date.month(), gps.date.day(),
      gps.time.hour(), gps.time.minute(), gps.time.second()
    );
  } else {
    // Fallback: nome sequencial
    int i = 0;
    do {
      sprintf(filename, "/%s_%03d.csv", prefixo.c_str(), i++);
    } while (SD.exists(filename) && i < 999);
  }

  nomeArquivo = String(filename);
  logFile = SD.open(nomeArquivo, FILE_WRITE);

  if (!logFile) {
    Serial.println("[SD] ERRO CRÍTICO: Falha ao criar arquivo de log!");
    estadoAtual = ESTADO_ERRO;
    return;
  }

  // Cabeçalho do CSV — varia conforme o modo
  // Metadados de identificação
  logFile.print("# APUAMA RACING | Modo: ");
  logFile.print(prefixo);
  logFile.print(" | Largada: ");
  logFile.print(pontoLargada.lat, 3);
  logFile.print(",");
  logFile.println(pontoLargada.lon, 3);

  // Colunas de dados
  switch (modoAtual) {
    case MODO_ACCELERATION:
      logFile.println("timestamp_ms,velocidade_kmh,distancia_m,tempo_prova_ms");
      break;
    case MODO_AUTOCROSS:
      logFile.println("timestamp_ms,lat,lon,velocidade_kmh,tempo_prova_ms");
      break;
    case MODO_ENDURO:
      logFile.println("timestamp_ms,lat,lon,velocidade_kmh,volta,tempo_volta_ms,tempo_total_ms");
      break;
    default:
      break;
  }

  logFile.flush();
  Serial.print("[SD] Arquivo criado: "); Serial.println(nomeArquivo);
}

// Grava uma linha de dados no CSV conforme o modo ativo
void gravarLinha() {
  if (!logFile) return;

  unsigned long agora = millis();

  switch (modoAtual) {

    case MODO_ACCELERATION: {
      // Distância calculada diretamente do ponto de início até a posição atual.
      // Evita Random Walk: somar incrementos de 20Hz acumula o ruído do GPS (~1m CEP),
      // fazendo o sistema achar que o carro percorreu mais do que realmente andou.
      // Como a prova é uma reta, Haversine(início, agora) é geometricamente correto.
      double distanciaAtual_m = calcularDistancia_m(
        pontoInicioAcel.lat, pontoInicioAcel.lon,
        gps.location.lat(),  gps.location.lng()
      );

      logFile.print(agora);                          logFile.print(",");
      logFile.print(gps.speed.kmph(), 2);            logFile.print(",");
      logFile.print(distanciaAtual_m, 2);            logFile.print(",");
      logFile.println(agora - tempoInicio_ms);
      break;
    }

    case MODO_AUTOCROSS: {
      logFile.print(agora);                          logFile.print(",");
      logFile.print(gps.location.lat(), 3);          logFile.print(",");
      logFile.print(gps.location.lng(), 3);          logFile.print(",");
      logFile.print(gps.speed.kmph(), 2);            logFile.print(",");
      logFile.println(agora - tempoInicio_ms);
      break;
    }

    case MODO_ENDURO: {
      logFile.print(agora);                          logFile.print(",");
      logFile.print(gps.location.lat(), 3);          logFile.print(",");
      logFile.print(gps.location.lng(), 3);          logFile.print(",");
      logFile.print(gps.speed.kmph(), 2);            logFile.print(",");
      logFile.print(numeroVolta);                    logFile.print(",");
      logFile.print(agora - tempoVoltaInicio_ms);    logFile.print(",");
      logFile.println(agora - tempoInicio_ms);
      break;
    }

    default: break;
  }

  // Flush periódico
  contadorFlush++;
  if (contadorFlush >= LINHAS_POR_FLUSH) {
    logFile.flush();
    contadorFlush = 0;
  }
}

// Verifica se o GPS tem qualidade suficiente para operar
bool fixValido() {
  return gps.location.isValid()
      && gps.satellites.isValid()
      && gps.hdop.isValid()
      && gps.satellites.value() >= MIN_SATELITES
      && gps.hdop.hdop()        <= MAX_HDOP;
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n========================================");
  Serial.println("  APUAMA RACING - GPS v1.0");
  Serial.println("========================================");

  // Habilita o módulo GNSS
  pinMode(GPS_EN_PIN, OUTPUT);
  digitalWrite(GPS_EN_PIN, HIGH);
  delay(500);

  // Inicia UART com o GPS (já configurado para 115200 via u-center)
  SerialGPS.begin(115200, SERIAL_8N1, RX_PIN_GPS, TX_PIN_GPS);
  delay(1000);

  // Configura GPS via UBX
  configurarGPS();

  // Inicia SD
  Serial.print("[SD] Iniciando... ");
  if (!SD.begin(CS_PIN_SD)) {
    Serial.println("FALHA! Verifique o cartão SD.");
    estadoAtual = ESTADO_ERRO;
    return;
  }
  Serial.println("OK.");

  // Lê modo de prova
  modoAtual = lerModoDoSD();
  if (modoAtual == MODO_DESCONHECIDO) {
    estadoAtual = ESTADO_ERRO;
    return;
  }

  // Tudo OK, aguardar fix
  estadoAtual = ESTADO_AGUARDANDO_FIX;
  Serial.println("[SYS] Aguardando fix de qualidade...");
  Serial.print("[SYS] Exigindo >= "); Serial.print(MIN_SATELITES);
  Serial.print(" satélites e HDOP <= "); Serial.println(MAX_HDOP);
}

// =============================================================================
// LOOP — MÁQUINA DE ESTADOS
// =============================================================================

void loop() {

  // Alimenta o parser do TinyGPSPlus com os bytes recebidos
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }

  // -------------------------------------------------------------------------
  switch (estadoAtual) {

    // -----------------------------------------------------------------------
    case ESTADO_AGUARDANDO_FIX: {
      static unsigned long ultimoLog = 0;
      if (millis() - ultimoLog > 2000) {
        ultimoLog = millis();
        Serial.print("[FIX] Satélites: ");
        Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
        Serial.print(" | HDOP: ");
        Serial.println(gps.hdop.isValid() ? gps.hdop.hdop() : 99.9);
      }

      if (fixValido()) {
        Serial.println("[FIX] Fix de qualidade obtido!");
        estadoAtual = ESTADO_CAPTURANDO_LARGADA;
      }
      break;
    }

    // -----------------------------------------------------------------------
    case ESTADO_CAPTURANDO_LARGADA: {
      // Captura as coordenadas atuais como ponto de largada
      pontoLargada.lat = gps.location.lat();
      pontoLargada.lon = gps.location.lng();
      largadaCapturada = true;

      Serial.print("[LARGADA] Ponto capturado: ");
      Serial.print(pontoLargada.lat, 3); Serial.print(", ");
      Serial.println(pontoLargada.lon, 3);

      // Cria o arquivo CSV (usa timestamp do GPS no nome)
      criarArquivoCSV();
      if (estadoAtual == ESTADO_ERRO) break; // criarArquivoCSV pode setar ERRO

      // Inicia janela de espera
      tempoJanela_ms = millis();
      estadoAtual = ESTADO_JANELA_LARGADA;

      Serial.print("[SYS] Aguardando ");
      Serial.print(JANELA_LARGADA_MS / 1000);
      Serial.println("s antes de iniciar a prova...");
      break;
    }

    // -----------------------------------------------------------------------
    case ESTADO_JANELA_LARGADA: {
      // Conta regressiva no monitor serial a cada segundo
      static unsigned long ultimoLog = 0;
      unsigned long restante = JANELA_LARGADA_MS - (millis() - tempoJanela_ms);

      if (millis() - ultimoLog > 1000) {
        ultimoLog = millis();
        Serial.print("[SYS] Iniciando em ");
        Serial.print(restante / 1000 + 1);
        Serial.println("s...");
      }

      if (millis() - tempoJanela_ms >= JANELA_LARGADA_MS) {
        estadoAtual = ESTADO_AGUARDANDO_INICIO;
        Serial.println("[SYS] Pronto! Aguardando início da prova...");
      }
      break;
    }

    // -----------------------------------------------------------------------
    case ESTADO_AGUARDANDO_INICIO: {

      // Sem dado novo do GPS, não faz nada
      if (!gps.location.isUpdated()) break;

      // --- ACCELERATION: trigger por velocidade ---
      if (modoAtual == MODO_ACCELERATION) {
        if (gps.speed.kmph() >= VEL_INICIO_ACCEL_KMH) {
          tempoInicio_ms      = millis();
          // Captura posição exata do início — referência fixa para cálculo de distância
          pontoInicioAcel.lat = gps.location.lat();
          pontoInicioAcel.lon = gps.location.lng();
          estadoAtual         = ESTADO_GRAVANDO;
          Serial.println("[ACCEL] Início detectado! Cronômetro rodando...");
        }
      }

      // --- AUTOCROSS / ENDURO: trigger por geofence ---
      else if (modoAtual == MODO_AUTOCROSS || modoAtual == MODO_ENDURO) {
        double distLargada = calcularDistancia_m(
          gps.location.lat(), gps.location.lng(),
          pontoLargada.lat, pontoLargada.lon
        );

        // Carro saiu da área de largada → pode cronometrar o retorno
        if (!saiuDoGeofence && distLargada > DIST_SAIDA_GEOFENCE) {
          saiuDoGeofence = true;
          tempoInicio_ms = millis();
          tempoVoltaInicio_ms = millis();
          numeroVolta = 1;
          estadoAtual = ESTADO_GRAVANDO;
          Serial.println("[PROVA] Carro saiu da largada! Cronômetro rodando...");
        }
      }
      break;
    }

    // -----------------------------------------------------------------------
    case ESTADO_GRAVANDO: {

      if (!gps.location.isUpdated()) break;

      // Grava linha de dados
      gravarLinha();

      // --- Lógica de encerramento por modo ---

      if (modoAtual == MODO_ACCELERATION) {
        double distanciaAtual_m = calcularDistancia_m(
          pontoInicioAcel.lat, pontoInicioAcel.lon,
          gps.location.lat(),  gps.location.lng()
        );
        // Encerra quando distância direta do ponto de início >= 75m
        if (distanciaAtual_m >= DIST_PROVA_ACCEL_M) {
          tempoFim_ms = millis();
          logFile.flush();
          logFile.close();
          estadoAtual = ESTADO_FINALIZADO;

          Serial.println("[ACCEL] PROVA ENCERRADA!");
          Serial.print("[ACCEL] Tempo: ");
          Serial.print((tempoFim_ms - tempoInicio_ms) / 1000.0, 3);
          Serial.println("s");
        }
      }

      else if (modoAtual == MODO_AUTOCROSS) {
        // Encerra quando retorna ao geofence de largada
        double distLargada = calcularDistancia_m(
          gps.location.lat(), gps.location.lng(),
          pontoLargada.lat, pontoLargada.lon
        );

        if (distLargada <= RAIO_GEOFENCE_M) {
          tempoFim_ms = millis();
          logFile.flush();
          logFile.close();
          estadoAtual = ESTADO_FINALIZADO;

          Serial.println("[AUTO] VOLTA ENCERRADA!");
          Serial.print("[AUTO] Tempo: ");
          Serial.print((tempoFim_ms - tempoInicio_ms) / 1000.0, 3);
          Serial.println("s");
        }
      }

      else if (modoAtual == MODO_ENDURO) {
        // Registra nova volta a cada retorno ao geofence
        double distLargada = calcularDistancia_m(
          gps.location.lat(), gps.location.lng(),
          pontoLargada.lat, pontoLargada.lon
        );

        // Detecta cruzamento da linha de chegada/largada
        // Usa uma flag para evitar múltiplos disparos enquanto no geofence
        static bool dentroDoGeofence = false;

        if (!dentroDoGeofence && distLargada <= RAIO_GEOFENCE_M) {
          dentroDoGeofence = true;
          unsigned long agora = millis();
          unsigned long tempoVolta = agora - tempoVoltaInicio_ms;
          tempoVoltaInicio_ms = agora;

          Serial.print("[ENDURO] Volta "); Serial.print(numeroVolta);
          Serial.print(" completa! Tempo: ");
          Serial.print(tempoVolta / 1000.0, 3); Serial.println("s");

          numeroVolta++;
        } else if (dentroDoGeofence && distLargada > RAIO_GEOFENCE_M) {
          dentroDoGeofence = false; // Saiu do geofence, pode detectar próxima volta
        }
        // Enduro não tem encerramento automático — o operador desliga o dispositivo
        // O flush periódico em gravarLinha() garante que os dados estão salvos
      }

      break;
    }

    // -----------------------------------------------------------------------
    case ESTADO_FINALIZADO: {
      // Estado terminal para Acceleration e Autocross.
      // Imprime mensagem uma única vez e fica parado.
      static bool mensagemExibida = false;
      if (!mensagemExibida) {
        mensagemExibida = true;
        Serial.println("[SYS] Sessão finalizada. Arquivo salvo em: " + nomeArquivo);
        Serial.println("[SYS] Pode desligar o dispositivo.");
      }
      break;
    }

    // -----------------------------------------------------------------------
    case ESTADO_ERRO: {
      // Estado de erro crítico — pisca aviso no monitor a cada 3s
      static unsigned long ultimoAviso = 0;
      if (millis() - ultimoAviso > 3000) {
        ultimoAviso = millis();
        Serial.println("[ERRO] Sistema em estado de erro. Verifique SD e config.txt.");
      }
      break;
    }

  } // fim switch
}
