#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// --- Display OLED ---
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

// --- DHT11 ---
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// --- Sensor de Chuva ---
#define PINO_CHUVA A0
const int LIMIAR_CHUVA = 100;

// --- Módulo Bluetooth BLE / HC-05 (RX: 10, TX: 11) ---
SoftwareSerial bluetooth(10, 11);
Adafruit_BMP280 bmp;

const float ALTITUDE_LOCAL = 780.0;

// --- Temporizadores ---
unsigned long agora = 0;
unsigned long ultimoEnvio = 0;
const unsigned long INTERVALO_ENVIO = 3000UL;

unsigned long ultimaTrocaPagina = 0;
const unsigned long INTERVALO_PAGINA = 4000UL;
byte paginaAtual = 0;

// --- Buffer Barométrico (3 Horas) ---
const int TOTAL_AMOSTRAS = 12;
const unsigned long INTERVALO_HISTORICO = 10000UL; // Ajuste para bancada ou 900000UL para 15 min reais
float historicoPressao[TOTAL_AMOSTRAS];
int indiceBuffer = 0;
int amostrasColetadas = 0;
unsigned long ultimoRegistroHistorico = 0;

// --- Buffer do Pager (Recepção Segura sem travar loop) ---
char rxBufferBT[64];
byte rxIndexBT = 0;
char textoTratado[64] = "";
char textoRolagem[80] = "";
int posRolagem = 0;
unsigned long ultimoPassoRolagem = 0;
const unsigned long VELOCIDADE_ROLAGEM = 180UL;
unsigned long tempoInicioMensagem = 0;
const unsigned long TEMPO_MINIMO_MENSAGEM = 8000UL;
bool exibindoMensagem = false;

struct DadosMeteo {
  float temp;
  float umi;
  float presLocal;
  float qnh;
  float deltaP;
  float pontoOrvalho;
  float baseNuvens;
  float sensacao;
  bool riscoGeada;
  const char* simboloTendencia;
  const char* descTendencia;
  const char* zambretti;
};

// Sanitização direta em buffer de caracteres
void sanitizarCStr(const char* entrada, char* saida, size_t maxLen) {
  size_t outIdx = 0;
  size_t inLen = strlen(entrada);

  for (size_t i = 0; i < inLen && outIdx < maxLen - 1; i++) {
    uint8_t c = (uint8_t)entrada[i];
    if (c == 0xC3 && i + 1 < inLen) {
      i++;
      uint8_t c2 = (uint8_t)entrada[i];
      if (c2 >= 0x80 && c2 <= 0x85)      saida[outIdx++] = 'A';
      else if (c2 == 0x87)               saida[outIdx++] = 'C';
      else if (c2 >= 0x88 && c2 <= 0x8B) saida[outIdx++] = 'E';
      else if (c2 >= 0x8C && c2 <= 0x8F) saida[outIdx++] = 'I';
      else if (c2 >= 0x92 && c2 <= 0x96) saida[outIdx++] = 'O';
      else if (c2 >= 0x99 && c2 <= 0x9C) saida[outIdx++] = 'U';
      else if (c2 >= 0xA0 && c2 <= 0xA5) saida[outIdx++] = 'a';
      else if (c2 == 0xA7)               saida[outIdx++] = 'c';
      else if (c2 >= 0xA8 && c2 <= 0xAB) saida[outIdx++] = 'e';
      else if (c2 >= 0xAC && c2 <= 0xAF) saida[outIdx++] = 'i';
      else if (c2 >= 0xB2 && c2 <= 0xB6) saida[outIdx++] = 'o';
      else if (c2 >= 0xB9 && c2 <= 0xBC) saida[outIdx++] = 'u';
      else                               saida[outIdx++] = ' ';
    } else if (c >= 32 && c <= 126) {
      saida[outIdx++] = (char)c;
    }
  }
  saida[outIdx] = '\0';
}

float calcularPontoOrvalho(float T, float RH) {
  if (RH <= 0.0) RH = 1.0;
  float a = 17.27;
  float b = 237.7;
  float alpha = ((a * T) / (b + T)) + log(RH / 100.0);
  return (b * alpha) / (a - alpha);
}

float calcularBaseNuvens(float T, float Td) {
  float base = 125.0 * (T - Td);
  return (base < 0.0) ? 0.0 : base;
}

float calcularSensacao(float T, float RH) {
  if (T < 27.0 || RH < 40.0) return T;
  return -8.784 + (1.611 * T) + (2.338 * RH) - (0.146 * T * RH)
         - (0.0123 * T * T) - (0.0164 * RH * RH) + (0.0022 * T * T * RH);
}

float calcularQNH(float pEstacao, float T, float altitude) {
  return pEstacao * pow(1.0 - ((0.0065 * altitude) / (T + (0.0065 * altitude) + 273.15)), -5.257);
}

const char* calcularZambretti(float qnh, float deltaP) {
  if (deltaP <= -2.0) {
    return (qnh < 1010.0) ? "Tempestade Iminente" : "Chuva e Vento";
  } else if (deltaP <= -0.8) {
    return (qnh < 1013.0) ? "Chuva Provavel" : "Instavel";
  } else if (deltaP >= 2.0) {
    return (qnh > 1020.0) ? "Firme e Seco" : "Tempo Limpando";
  } else if (deltaP >= 0.8) {
    return (qnh > 1015.0) ? "Melhoria Continua" : "Tempo Bom";
  } else {
    if (qnh >= 1020.0) return "Ensolarado Firme";
    if (qnh >= 1012.0) return "Tempo Estavel";
    return "Nublado / Variavel";
  }
}

DadosMeteo processarMeteorologia() {
  DadosMeteo d;
  d.temp = bmp.readTemperature();
  d.presLocal = bmp.readPressure() / 100.0F;
  d.umi = dht.readHumidity();
  if (isnan(d.umi)) d.umi = 0.0;

  d.qnh = calcularQNH(d.presLocal, d.temp, ALTITUDE_LOCAL);

  if (amostrasColetadas < 2) {
    d.deltaP = 0.0;
    d.simboloTendencia = "[..]";
    d.descTendencia = "Coletando";
  } else {
    int idxMaisAntigo = (amostrasColetadas == TOTAL_AMOSTRAS) ? indiceBuffer : 0;
    d.deltaP = d.presLocal - historicoPressao[idxMaisAntigo];

    if (d.deltaP <= -2.0) {
      d.simboloTendencia = "[vv]";
      d.descTendencia = "Queda Brusca";
    } else if (d.deltaP <= -0.8) {
      d.simboloTendencia = "[ v]";
      d.descTendencia = "Queda Moderada";
    } else if (d.deltaP >= 2.0) {
      d.simboloTendencia = "[^^]";
      d.descTendencia = "Subida Rapida";
    } else if (d.deltaP >= 0.8) {
      d.simboloTendencia = "[ ^]";
      d.descTendencia = "Subida Gradual";
    } else {
      d.simboloTendencia = "[ =]";
      d.descTendencia = "Estavel";
    }
  }

  d.pontoOrvalho = calcularPontoOrvalho(d.temp, d.umi);
  d.baseNuvens = calcularBaseNuvens(d.temp, d.pontoOrvalho);
  d.sensacao = calcularSensacao(d.temp, d.umi);
  d.riscoGeada = (d.temp <= 4.0 && d.pontoOrvalho <= 0.0);
  d.zambretti = calcularZambretti(d.qnh, d.deltaP);

  return d;
}

void desenharPagina0(const DadosMeteo &d, int leituraChuva, const char* statusChuva) {
  oled.home();
  oled.set1X();
  oled.println("--- METEO ERECHIM ---");

  oled.set2X();
  oled.print(d.temp, 1);
  oled.println(" C");

  oled.set1X();
  oled.print("Umid: ");
  oled.print(d.umi, 0);
  oled.println(" %");

  oled.print("P.Local: ");
  oled.print(d.presLocal, 1);
  oled.print(" ");
  oled.println(d.simboloTendencia);

  oled.print("QNH:     ");
  oled.print(d.qnh, 1);
  oled.println(" hPa");

  oled.print("Chuva: ");
  oled.print(statusChuva);
  oled.print(" (");
  oled.print(leituraChuva);
  oled.println(") ");

  oled.setCursor(110, 7);
  oled.print("1/2");
}

void desenharPagina1(const DadosMeteo &d) {
  oled.home();
  oled.set1X();
  oled.println("--- PREVISAO / IND ---");

  oled.print("Zamb: ");
  oled.println(d.zambretti);

  oled.print("Orvalho:  ");
  oled.print(d.pontoOrvalho, 1);
  oled.println(" C");

  oled.print("Base Nuv: ");
  oled.print(d.baseNuvens, 0);
  oled.println(" m");

  oled.print("Sensacao: ");
  oled.print(d.sensacao, 1);
  oled.println(" C");

  oled.print("Geada:    ");
  oled.println(d.riscoGeada ? "!ALERTA!" : "Sem Risco");

  oled.print("Var 3h:   ");
  if (d.deltaP > 0) oled.print("+");
  oled.print(d.deltaP, 1);
  oled.println(" hPa");

  oled.setCursor(110, 7);
  oled.print("2/2");
}

// Transmissão Compacta CSV via BLE (Apenas ~60 bytes)
void transmitirBLE(const DadosMeteo &d, int leituraChuva) {
  bluetooth.print(F("$MET,"));
  bluetooth.print(d.temp, 1); bluetooth.print(',');
  bluetooth.print(d.umi, 0); bluetooth.print(',');
  bluetooth.print(d.presLocal, 1); bluetooth.print(',');
  bluetooth.print(d.qnh, 1); bluetooth.print(',');
  bluetooth.print(d.deltaP, 1); bluetooth.print(',');
  bluetooth.print(d.descTendencia); bluetooth.print(',');
  bluetooth.print(d.zambretti); bluetooth.print(',');
  bluetooth.print(d.pontoOrvalho, 1); bluetooth.print(',');
  bluetooth.print(d.baseNuvens, 0); bluetooth.print(',');
  bluetooth.print(d.sensacao, 1); bluetooth.print(',');
  bluetooth.print(d.riscoGeada ? 1 : 0); bluetooth.print(',');
  bluetooth.println(leituraChuva);
}

// Transmissão Formatada para o Monitor Serial USB (PC)
void transmitirSerial(const DadosMeteo &d, const char* statusChuva, int leituraChuva) {
  Serial.print(F("T: ")); Serial.print(d.temp, 1);
  Serial.print(F("C | Umid: ")); Serial.print(d.umi, 0);
  Serial.print(F("% | QNH: ")); Serial.print(d.qnh, 1);
  Serial.print(F("hPa | Zamb: ")); Serial.print(d.zambretti);
  Serial.print(F(" | Chuva: ")); Serial.print(statusChuva);
  Serial.print(F(" (")); Serial.print(leituraChuva); Serial.println(F(")"));
}

void setup() {
  Serial.begin(9600);
  bluetooth.begin(38400);
  Wire.begin();
  Wire.setClock(400000L);

  dht.begin();

  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.set1X();
  oled.println("ESTACAO COMPLETA");
  oled.println("METEO + PREVISAO");
  delay(800);
  oled.clear();

  if (!bmp.begin(0x76)) {
    Serial.println(F("Erro: BMP280!"));
    bluetooth.println(F("ERRO:BMP280"));
    oled.println("Erro: BMP280!");
    while (1);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  float pInicial = bmp.readPressure() / 100.0F;
  historicoPressao[0] = pInicial;
  indiceBuffer = 1;
  amostrasColetadas = 1;
  ultimoRegistroHistorico = millis();
}

void loop() {
  agora = millis();

  // 1. Buffer histórico barométrico
  if (agora - ultimoRegistroHistorico >= INTERVALO_HISTORICO) {
    ultimoRegistroHistorico = agora;
    historicoPressao[indiceBuffer] = bmp.readPressure() / 100.0F;
    indiceBuffer = (indiceBuffer + 1) % TOTAL_AMOSTRAS;
    if (amostrasColetadas < TOTAL_AMOSTRAS) amostrasColetadas++;
  }

  // 2. Recepção não-bloqueante do Pager via Bluetooth
  while (bluetooth.available()) {
    char c = (char)bluetooth.read();
    if (c == '\n' || c == '\r') {
      if (rxIndexBT > 0) {
        rxBufferBT[rxIndexBT] = '\0';
        sanitizarCStr(rxBufferBT, textoTratado, sizeof(textoTratado));
        rxIndexBT = 0;

        size_t len = strlen(textoTratado);
        if (len > 0) {
          exibindoMensagem = true;
          tempoInicioMensagem = agora;
          posRolagem = 0;

          if (len > 10) {
            snprintf(textoRolagem, sizeof(textoRolagem), "%s   ---   ", textoTratado);
          } else {
            strncpy(textoRolagem, textoTratado, sizeof(textoRolagem));
          }

          oled.clear();
          oled.set1X();
          oled.println(">> MENSAGEM RECEBIDA <<");
          oled.println("");

          bluetooth.print(F("ACK:"));
          bluetooth.println(textoTratado);
        }
      }
    } else if (rxIndexBT < sizeof(rxBufferBT) - 1) {
      rxBufferBT[rxIndexBT++] = c;
    }
  }

  // 3. Efeito letreiro no OLED
  if (exibindoMensagem) {
    if (agora - ultimoPassoRolagem >= VELOCIDADE_ROLAGEM) {
      ultimoPassoRolagem = agora;
      oled.setCursor(0, 3);
      oled.set2X();

      size_t lenMsg = strlen(textoTratado);
      if (lenMsg <= 10) {
        oled.clearToEOL();
        oled.print(textoTratado);
      } else {
        size_t lenRol = strlen(textoRolagem);
        char janela[11];
        for (int i = 0; i < 10; i++) {
          janela[i] = textoRolagem[(posRolagem + i) % lenRol];
        }
        janela[10] = '\0';
        oled.clearToEOL();
        oled.print(janela);
        posRolagem++;
      }

      if (agora - tempoInicioMensagem >= TEMPO_MINIMO_MENSAGEM) {
        size_t lenRol = strlen(textoRolagem);
        if (lenMsg <= 10 || (lenRol > 0 && (posRolagem % lenRol == 0))) {
          exibindoMensagem = false;
          oled.clear();
        }
      }
    }
  }

  // 4. Execução Meteorológica
  if (!exibindoMensagem) {
    DadosMeteo dados = processarMeteorologia();
    int leituraChuva = analogRead(PINO_CHUVA);
    const char* statusChuva = (leituraChuva > LIMIAR_CHUVA) ? "SIM!" : "NAO";

    if (agora - ultimaTrocaPagina >= INTERVALO_PAGINA) {
      ultimaTrocaPagina = agora;
      paginaAtual = (paginaAtual + 1) % 2;
      oled.clear();
    }

    if (paginaAtual == 0) {
      desenharPagina0(dados, leituraChuva, statusChuva);
    } else {
      desenharPagina1(dados);
    }

    // 5. Transmissão
    if (agora - ultimoEnvio >= INTERVALO_ENVIO) {
      ultimoEnvio = agora;
      transmitirBLE(dados, leituraChuva);
      transmitirSerial(dados, statusChuva, leituraChuva);
    }
  }
}