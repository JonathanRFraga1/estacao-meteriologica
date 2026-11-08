#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// --- Display OLED Ultraleve ---
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

// --- DHT11 (Umidade no Pino 2) ---
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// --- Sensor de Chuva / Nível de Água ---
#define PINO_CHUVA A0
const int LIMIAR_CHUVA = 100;

// --- HC-05 (RX: 10, TX: 11) a 38400 baud ---
SoftwareSerial bluetooth(10, 11);
Adafruit_BMP280 bmp;

const float PRESSAO_NIVEL_MAR = 1019.8; // Referência Erechim
unsigned long ultimoEnvio = 0;
const unsigned long INTERVALO_ENVIO = 2500UL;

// --- Buffer de Tendência Barométrica (Padrão OMM: 3 horas) ---
const int TOTAL_AMOSTRAS = 12; // 12 amostras * 15 min = 180 min (3 horas)
const unsigned long INTERVALO_HISTORICO = 900000UL; // 15 minutos em ms (use 10000UL para testes)
float historicoPressao[TOTAL_AMOSTRAS];
int indiceBuffer = 0;
int amostrasColetadas = 0;
unsigned long ultimoRegistroHistorico = 0;

// --- Variáveis de Mensagem e Rolagem (Pager) ---
String textoTratado = "";
String textoRolagem = "";
int posRolagem = 0;
unsigned long ultimoPassoRolagem = 0;
const unsigned long VELOCIDADE_ROLAGEM = 180UL;
unsigned long tempoInicioMensagem = 0;
const unsigned long TEMPO_MINIMO_MENSAGEM = 8000UL;
bool exibindoMensagem = false;

// Remove acentos UTF-8 do celular para o display ASCII
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
    } 
    else if (c >= 32 && c <= 126) {
      saida += (char)c;
    }
  }
  return saida;
}

// Analisa a tendência tri-horária e retorna índice e deltaP
void avaliarBarometro(float pAtual, float &deltaP, const char* &simboloOLED, String &relatorioBT) {
  if (amostrasColetadas < 2) {
    deltaP = 0.0;
    simboloOLED = "[..]";
    relatorioBT = F("Coletando historico inicial...");
    return;
  }

  int indiceMaisAntigo = (amostrasColetadas == TOTAL_AMOSTRAS) ? indiceBuffer : 0;
  deltaP = pAtual - historicoPressao[indiceMaisAntigo];

  if (deltaP <= -2.0) {
    simboloOLED = "[vv]";
    relatorioBT = "[ALERTA] Queda brusca (" + String(deltaP, 1) + " hPa) -> Temporal/Chuva forte!";
  } else if (deltaP <= -0.8) {
    simboloOLED = "[ v]";
    relatorioBT = "[AVISO] Queda moderada (" + String(deltaP, 1) + " hPa) -> Tempo instavel.";
  } else if (deltaP >= 2.0) {
    simboloOLED = "[^^]";
    relatorioBT = "[INFO] Subida rapida (+" + String(deltaP, 1) + " hPa) -> Frente fria / Tempo limpando.";
  } else if (deltaP >= 0.8) {
    simboloOLED = "[ ^]";
    relatorioBT = "[INFO] Subida gradual (+" + String(deltaP, 1) + " hPa) -> Melhoria do tempo.";
  } else {
    simboloOLED = "[ =]";
    relatorioBT = "[ESTAVEL] Sem alteracoes significativas (" + String(deltaP, 1) + " hPa).";
  }
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
  oled.println("INICIANDO...");
  delay(1000);
  oled.clear();

  if (!bmp.begin(0x76)) {
    Serial.println(F("Erro: BMP280 nao detectado!"));
    bluetooth.println(F("Erro: BMP280 nao detectado!"));
    oled.println("Erro: BMP280!");
    while (1);
  }

  // Ativa filtro IIR de hardware (filtra rajadas de vento e portas batendo)
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,    // Oversampling temperatura
                  Adafruit_BMP280::SAMPLING_X16,   // Oversampling pressao (max precisao)
                  Adafruit_BMP280::FILTER_X16,     // Filtro IIR interno
                  Adafruit_BMP280::STANDBY_MS_500);

  // Inicializa buffer histórico com a leitura atual
  float pInicial = bmp.readPressure() / 100.0F;
  historicoPressao[0] = pInicial;
  indiceBuffer = 1;
  amostrasColetadas = 1;
  ultimoRegistroHistorico = millis();

  bluetooth.println(F("--- Estacao Conectada: Meteo + Barometro 3h + Pager ---"));
}

void loop() {
  unsigned long agora = millis();

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

  // 2. Leitura de mensagens enviadas pelo celular via Bluetooth
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

  // 3. Animação de Rolagem da Mensagem (Pager)
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

  // 4. Atualização Meteorológica Local e Envio Remoto
  if (!exibindoMensagem && (agora - ultimoEnvio >= INTERVALO_ENVIO)) {
    ultimoEnvio = agora;

    float temp = bmp.readTemperature();
    float pres = bmp.readPressure() / 100.0F;
    float alt  = bmp.readAltitude(PRESSAO_NIVEL_MAR);
    float umi  = dht.readHumidity();
    int leituraChuva = analogRead(PINO_CHUVA);

    if (isnan(umi)) umi = 0.0;

    String statusChuva = (leituraChuva > LIMIAR_CHUVA) ? "SIM!" : "NAO";

    // Cálculo da tendência barométrica
    float deltaP = 0.0;
    const char* simboloOLED = "[ =]";
    String relatorioBT = "";
    avaliarBarometro(pres, deltaP, simboloOLED, relatorioBT);

    // --- Atualização da Tela OLED ---
    oled.home();
    oled.set1X();
    oled.println("--- METEO ERECHIM ---");

    oled.set2X();
    oled.print(temp, 1);
    oled.println(" C");

    oled.set1X();
    oled.print("Umid: ");
    oled.print(umi, 0);
    oled.println(" %");

    // Linha de pressão com indicador compacto de tendência
    oled.print("Pres: ");
    oled.print(pres, 1);
    oled.print(" ");
    oled.println(simboloOLED);

    oled.print("Alt:  ");
    oled.print(alt, 1);
    oled.println(" m");

    oled.print("Chuva: ");
    oled.print(statusChuva);
    oled.print(" (");
    oled.print(leituraChuva);
    oled.println(") ");

    // --- Telemetria Bluetooth com Detalhes da Tendência ---
    bluetooth.println(F("================================"));
    bluetooth.print(F("Temp:     ")); bluetooth.print(temp, 1); bluetooth.println(F(" C"));
    bluetooth.print(F("Umidade:  ")); bluetooth.print(umi, 0); bluetooth.println(F(" %"));
    bluetooth.print(F("Pressao:  ")); bluetooth.print(pres, 1); bluetooth.println(F(" hPa"));
    bluetooth.print(F("Alt:      ")); bluetooth.print(alt, 1); bluetooth.println(F(" m"));
    bluetooth.print(F("Tendencia:")); bluetooth.println(relatorioBT);
    bluetooth.print(F("Chuva:    ")); bluetooth.print(statusChuva);
    bluetooth.print(F(" (raw: ")); bluetooth.print(leituraChuva); bluetooth.println(F(")"));
  }
}