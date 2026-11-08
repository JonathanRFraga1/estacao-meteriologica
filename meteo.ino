#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// --- Display OLED Ultraleve ---
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

// --- DHT11 (Pino 2) ---
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// --- Sensor de Chuva / Nível de Água ---
#define PINO_CHUVA A0
const int LIMIAR_CHUVA = 100;

// --- Módulo Bluetooth BLE / HC-05 (RX: 10, TX: 11) a 38400 baud ---
SoftwareSerial bluetooth(10, 11);
Adafruit_BMP280 bmp;

// Altitude local de referência (Erechim/RS)
const float ALTITUDE_LOCAL = 780.0;

// --- Temporizadores e Intervalos ---
unsigned long agora = 0;

unsigned long ultimoEnvioBT = 0;
const unsigned long INTERVALO_ENVIO_BT = 3000UL; // Telemetria a cada 3s

// Alternância de páginas no OLED (4 segundos por tela)
unsigned long ultimaTrocaPagina = 0;
const unsigned long INTERVALO_PAGINA = 4000UL;
byte paginaAtual = 0; // 0 = Tempo Real, 1 = Previsão e Índices

// --- Buffer de Tendência Barométrica (Padrão OMM: 3 horas) ---
const int TOTAL_AMOSTRAS = 12; // 12 leituras de 15 min = 180 min (3h)
const unsigned long INTERVALO_HISTORICO = 900000UL; // 15 minutos em ms (use 10000UL para testes de bancada)
float historicoPressao[TOTAL_AMOSTRAS];
int indiceBuffer = 0;
int amostrasColetadas = 0;
unsigned long ultimoRegistroHistorico = 0;

// --- Variáveis do Pager (Recados via Celular) ---
String textoTratado = "";
String textoRolagem = "";
int posRolagem = 0;
unsigned long ultimoPassoRolagem = 0;
const unsigned long VELOCIDADE_ROLAGEM = 180UL;
unsigned long tempoInicioMensagem = 0;
const unsigned long TEMPO_MINIMO_MENSAGEM = 8000UL;
bool exibindoMensagem = false;

// Estrutura que agrupa todas as previsões meteorológicas
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

// Filtro de caracteres acentuados UTF-8 vindos do teclado do celular
String sanitizarTexto(String entrada) {
  String saida = "";
  int len = entrada.length();

  for (int i = 0; i < len; i++) {
    uint8_t c = (uint8_t)entrada[i];

    if (c == 0xC3 && i + 1 < len) {
      i++;
      uint8_t c2 = (uint8_t)entrada[i];

      if (c2 >= 0x80 && c2 <= 0x85) saida += 'A';
      else if (c2 == 0x87)          saida += 'C';
      else if (c2 >= 0x88 && c2 <= 0x8B) saida += 'E';
      else if (c2 >= 0x8C && c2 <= 0x8F) saida += 'I';
      else if (c2 >= 0x92 && c2 <= 0x96) saida += 'O';
      else if (c2 >= 0x99 && c2 <= 0x9C) saida += 'U';
      else if (c2 >= 0xA0 && c2 <= 0xA5) saida += 'a';
      else if (c2 == 0xA7)          saida += 'c';
      else if (c2 >= 0xA8 && c2 <= 0xAB) saida += 'e';
      else if (c2 >= 0xAC && c2 <= 0xAF) saida += 'i';
      else if (c2 >= 0xB2 && c2 <= 0xB6) saida += 'o';
      else if (c2 >= 0xB9 && c2 <= 0xBC) saida += 'u';
      else saida += ' ';
    } else if (c >= 32 && c <= 126) {
      saida += (char)c;
    }
  }
  return saida;
}

// --- Funções Físico-Matemáticas de Meteorologia ---

// Fórmula de Magnus-Tetens para Ponto de Orvalho
float calcularPontoOrvalho(float T, float RH) {
  if (RH <= 0.0) RH = 1.0;
  float a = 17.27;
  float b = 237.7;
  float alpha = ((a * T) / (b + T)) + log(RH / 100.0);
  return (b * alpha) / (a - alpha);
}

// Equação de Espy para altitude da base de nuvens cumulus (metros)
float calcularBaseNuvens(float T, float Td) {
  float base = 125.0 * (T - Td);
  return (base < 0.0) ? 0.0 : base;
}

// Índice de calor / sensação térmica (NOAA)
float calcularSensacao(float T, float RH) {
  if (T < 27.0 || RH < 40.0) return T;
  return -8.784 + 1.611 * T + 2.338 * RH - 0.146 * T * RH 
         - 0.0123 * (T * T) - 0.0164 * (RH * RH) + 0.0022 * (T * T) * RH;
}

// Redução barométrica ao nível do mar (QNH internacional)
float calcularQNH(float pEstacao, float T, float altitude) {
  return pEstacao * pow(1.0 - ((0.0065 * altitude) / (T + (0.0065 * altitude) + 273.15)), -5.257);
}

// Algoritmo de Zambretti (Previsão de 12 a 24 horas)
const char* calcularZambretti(float qnh, float deltaP) {
  if (deltaP <= -2.0) {
    return (qnh < 1010.0) ? "Tempestade Iminente" : "Chuva e Vento Forte";
  } else if (deltaP <= -0.8) {
    return (qnh < 1013.0) ? "Chuva Provavel" : "Tempo Instavel";
  } else if (deltaP >= 2.0) {
    return (qnh > 1020.0) ? "Tempo Firme e Seco" : "Tempo Limpando";
  } else if (deltaP >= 0.8) {
    return (qnh > 1015.0) ? "Melhoria Continua" : "Tempo Bom";
  } else {
    if (qnh >= 1020.0) return "Ensolarado Firme";
    if (qnh >= 1012.0) return "Tempo Estavel";
    return "Nublado / Variavel";
  }
}

// Consolidação de todas as análises
DadosMeteo processarMeteorologia() {
  DadosMeteo d;
  d.temp = bmp.readTemperature();
  d.presLocal = bmp.readPressure() / 100.0F;
  d.umi = dht.readHumidity();
  if (isnan(d.umi)) d.umi = 0.0;

  d.qnh = calcularQNH(d.presLocal, d.temp, ALTITUDE_LOCAL);

  // Análise da variação do buffer de 3h
  if (amostrasColetadas < 2) {
    d.deltaP = 0.0;
    d.simboloTendencia = "[..]";
    d.descTendencia = "Coletando historico...";
  } else {
    int idxMaisAntigo = (amostrasColetadas == TOTAL_AMOSTRAS) ? indiceBuffer : 0;
    d.deltaP = d.presLocal - historicoPressao[idxMaisAntigo];

    if (d.deltaP <= -2.0) {
      d.simboloTendencia = "[vv]";
      d.descTendencia = "Queda Brusca (Alerta)";
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

// --- Funções de Renderização no Display OLED ---

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
  oled.println(d.riscoGeada ? "!ALERTA RISCO!" : "Sem Risco");

  oled.print("Var 3h:   ");
  if (d.deltaP > 0) oled.print("+");
  oled.print(d.deltaP, 1);
  oled.println(" hPa");

  oled.setCursor(110, 7);
  oled.print("2/2");
}

void transmitirTelemetria(Print &porta, const DadosMeteo &d, const char* statusChuva, int leituraChuva) {
  porta.println(F("========================================"));
  porta.print(F("Temp:       ")); porta.print(d.temp, 1); porta.println(F(" C"));
  porta.print(F("Umidade:    ")); porta.print(d.umi, 0); porta.println(F(" %"));
  porta.print(F("P. Local:   ")); porta.print(d.presLocal, 1); porta.println(F(" hPa"));
  porta.print(F("P. QNH:     ")); porta.print(d.qnh, 1); porta.println(F(" hPa"));
  porta.print(F("Var 3h:     ")); porta.print(d.deltaP, 1); porta.println(F(" hPa"));
  porta.print(F("Tendencia:  ")); porta.println(d.descTendencia);
  porta.print(F("Zambretti:  ")); porta.println(d.zambretti);
  porta.print(F("T. Orvalho: ")); porta.print(d.pontoOrvalho, 1); porta.println(F(" C"));
  porta.print(F("Base Nuvens:")); porta.print(d.baseNuvens, 0); porta.println(F(" m"));
  porta.print(F("Sensacao:   ")); porta.print(d.sensacao, 1); porta.println(F(" C"));
  porta.print(F("Geada:      ")); porta.println(d.riscoGeada ? F("SIM (ALERTA)") : F("NAO"));
  porta.print(F("Chuva:      ")); porta.print(statusChuva);
  porta.print(F(" (raw: ")); porta.print(leituraChuva); porta.println(F(")"));
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
  delay(1000);
  oled.clear();

  if (!bmp.begin(0x76)) {
    Serial.println(F("Erro: BMP280 nao detectado!"));
    bluetooth.println(F("Erro: BMP280 nao detectado!"));
    oled.println("Erro: BMP280!");
    while (1);
  }

  // Filtro de ruído IIR do BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  // Inicializa o buffer com a primeira leitura de bancada
  float pInicial = bmp.readPressure() / 100.0F;
  historicoPressao[0] = pInicial;
  indiceBuffer = 1;
  amostrasColetadas = 1;
  ultimoRegistroHistorico = millis();

  bluetooth.println(F("--- Estacao Online: Sensores + Zambretti + Pager ---"));
}

void loop() {
  agora = millis();

  // 1. Gravação no buffer circular a cada 15 minutos
  if (agora - ultimoRegistroHistorico >= INTERVALO_HISTORICO) {
    ultimoRegistroHistorico = agora;
    float pressaoInst = bmp.readPressure() / 100.0F;
    historicoPressao[indiceBuffer] = pressaoInst;
    indiceBuffer = (indiceBuffer + 1) % TOTAL_AMOSTRAS;

    if (amostrasColetadas < TOTAL_AMOSTRAS) {
      amostrasColetadas++;
    }
  }

  // 2. Recepção de mensagens Bluetooth (Pager)
  if (bluetooth.available()) {
    String msgBruta = bluetooth.readStringUntil('\n');
    msgBruta.trim();

    if (msgBruta.length() > 0) {
      textoTratado = sanitizarTexto(msgBruta);
      exibindoMensagem = true;
      tempoInicioMensagem = agora;
      posRolagem = 0;

      if (textoTratado.length() > 10) {
        textoRolagem = textoTratado + "   ---   ";
      } else {
        textoRolagem = textoTratado;
      }

      oled.clear();
      oled.set1X();
      oled.println(">> MENSAGEM RECEBIDA <<");
      oled.println("");

      bluetooth.print(F("Recebido: \""));
      bluetooth.print(textoTratado);
      bluetooth.println(F("\""));
    }
  }

  // 3. Efeito letreiro digital do Pager
  if (exibindoMensagem) {
    if (agora - ultimoPassoRolagem >= VELOCIDADE_ROLAGEM) {
      ultimoPassoRolagem = agora;

      oled.setCursor(0, 3);
      oled.set2X();

      if (textoTratado.length() <= 10) {
        oled.clearToEOL();
        oled.print(textoTratado);
      } else {
        String janela = "";
        for (int i = 0; i < 10; i++) {
          int idx = (posRolagem + i) % textoRolagem.length();
          janela += textoRolagem[idx];
        }
        oled.clearToEOL();
        oled.print(janela);
        posRolagem++;
      }

      if (agora - tempoInicioMensagem >= TEMPO_MINIMO_MENSAGEM) {
        if (textoTratado.length() <= 10 || (posRolagem % textoRolagem.length() == 0)) {
          exibindoMensagem = false;
          oled.clear();
        }
      }
    }
  }

  // 4. Execução Meteorológica: Telas do OLED e Telemetria
  if (!exibindoMensagem) {
    DadosMeteo dados = processarMeteorologia();
    int leituraChuva = analogRead(PINO_CHUVA);
    const char* statusChuva = (leituraChuva > LIMIAR_CHUVA) ? "SIM!" : "NAO";

    // Paginação do OLED (troca a cada 4 segundos sem piscar)
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

    // 5. Envio periódico para Bluetooth e Serial USB
    if (agora - ultimoEnvioBT >= INTERVALO_ENVIO_BT) {
      ultimoEnvioBT = agora;

      // Envia para o rádio BLE (celular/dashboard)
      transmitirTelemetria(bluetooth, dados, statusChuva, leituraChuva);

      // Espelha no cabo USB (Monitor Serial a 9600 baud)
      transmitirTelemetria(Serial, dados, statusChuva, leituraChuva);
    }
  }
}