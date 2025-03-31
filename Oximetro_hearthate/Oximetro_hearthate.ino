#include <Adafruit_GFX.h>    //Bibliotecas para OLED
#include <Adafruit_SSD1306.h> //Bibliotecas para OLED
#include "MAX30105.h"           //Biblioteca para MAX3010x
#include "heartRate.h"          //Algoritmo para cálculo da frequência cardíaca
//#include "ESP32Servo.h"
MAX30105 particleSensor;

//Variáveis para cálculo da frequência cardíaca
const byte RATE_SIZE = 8; //Quantidade de médias
byte rates[RATE_SIZE]; //Array de batimentos
byte rateSpot = 0;
long lastBeat = 0; //Tempo em que ocorreu o último batimento
float beatsPerMinute;
int beatAvg;

//Variáveis para cálculo de SpO2 (oxigenação do sangue)
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;

double SpO2 = 0;
double ESpO2 = 80.0; //Valor inicial
double FSpO2 = 0.8; //Fator de filtro para SpO2 estimado
double frate = 0.95; //Filtro passa-baixa para valores de LED IR/vermelho para eliminar componente AC
int i = 0;
int Num = 30; //Amostrar 30 vezes antes de calcular
#define FINGER_ON 7000    //Mínimo de infravermelho (para detectar se o dedo está presente)
#define MINIMUM_SPO2 60.0 //Mínimo de SpO2

//Configurações do OLED
#define SCREEN_WIDTH 128 //Largura do OLED
#define SCREEN_HEIGHT 64 //Altura do OLED
#define OLED_RESET    -1 //Pino de reset
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declarando o nome do display (display)

//Ícone pequeno de coração
static const unsigned char PROGMEM logo2_bmp[] =
{ 0x03, 0xC0, 0xF0, 0x06, 0x71, 0x8C, 0x0C, 0x1B, 0x06, 0x18, 0x0E, 0x02, 0x10, 0x0C, 0x03, 0x10,        
  0x04, 0x01, 0x10, 0x04, 0x01, 0x10, 0x40, 0x01, 0x10, 0x40, 0x01, 0x10, 0xC0, 0x03, 0x08, 0x88,
  0x02, 0x08, 0xB8, 0x04, 0xFF, 0x37, 0x08, 0x01, 0x30, 0x18, 0x01, 0x90, 0x30, 0x00, 0xC0, 0x60,
  0x00, 0x60, 0xC0, 0x00, 0x31, 0x80, 0x00, 0x1B, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x04, 0x00,
};
//Ícone grande de coração
static const unsigned char PROGMEM logo3_bmp[] =
{ 0x01, 0xF0, 0x0F, 0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
  0x20, 0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08, 0x03,
  0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x14, 0x00,
  0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40, 0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
  0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31, 0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
  0x00, 0x60, 0xE0, 0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
  0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C, 0x00,
  0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00
};
//Ícone de oxigênio
static const unsigned char PROGMEM O2_bmp[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x3f, 0xc3, 0xf8, 0x00, 0xff, 0xf3, 0xfc,
  0x03, 0xff, 0xff, 0xfe, 0x07, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0x7e,
  0x1f, 0x80, 0xff, 0xfc, 0x1f, 0x00, 0x7f, 0xb8, 0x3e, 0x3e, 0x3f, 0xb0, 0x3e, 0x3f, 0x3f, 0xc0,
  0x3e, 0x3f, 0x1f, 0xc0, 0x3e, 0x3f, 0x1f, 0xc0, 0x3e, 0x3f, 0x1f, 0xc0, 0x3e, 0x3e, 0x2f, 0xc0,
  0x3e, 0x3f, 0x0f, 0x80, 0x1f, 0x1c, 0x2f, 0x80, 0x1f, 0x80, 0xcf, 0x80, 0x1f, 0xe3, 0x9f, 0x00,
  0x0f, 0xff, 0x3f, 0x00, 0x07, 0xfe, 0xfe, 0x00, 0x0b, 0xfe, 0x0c, 0x00, 0x1d, 0xff, 0xf8, 0x00,
  0x1e, 0xff, 0xe0, 0x00, 0x1f, 0xff, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0x00, 0x1f, 0xe0, 0x00, 0x00,
  0x0f, 0xe0, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


void setup() {
  Serial.begin(115200);
  Serial.println("Sistema Iniciado");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Iniciar o display OLED
  display.display();
  delay(3000);
  //Verificação
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Usar a porta I2C padrão, velocidade de 400kHz
  {
    Serial.println("MAX30102 não encontrado");
    while (1);
  }
  //As opções a seguir podem ser ajustadas conforme necessário
  byte ledBrightness = 0x7F; //Brilho recomendado = 127, Opções: 0=Desligado até 255=50mA
  byte sampleAverage = 8; //Opções: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Opções: 1 = Somente vermelho (frequência cardíaca), 2 = Vermelho + IR (SpO2)
  int sampleRate = 3200; //Opções: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Opções: 69, 118, 215, 411
  int adcRange = 16384; //Opções: 2048, 4096, 8192, 16384
  // Configurar os parâmetros desejados
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configurar o sensor com essas configurações
  particleSensor.enableDIETEMPRDY();

  particleSensor.setPulseAmplitudeRed(0x0A); //Diminuir o brilho do LED vermelho para indicar que o sensor está funcionando

}

void loop() {
  long irValue = particleSensor.getIR();    //Ler o valor IR para saber se há um dedo no sensor ou não
  //Verificar se há um dedo no sensor
  if (irValue > FINGER_ON ) {
    
    //Verificar se há batimento cardíaco e medir a frequência cardíaca
    if (checkForBeat(irValue) == true) {
     
      long delta = millis() - lastBeat; //Calcular a diferença de tempo entre batimentos
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0); //Calcular a média de batimentos por minuto

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        //A frequência cardíaca deve estar entre 20 e 255

        rates[rateSpot++] = (byte)beatsPerMinute; //Armazenar o valor da frequência cardíaca no array
        rateSpot %= RATE_SIZE;
        beatAvg = 0; //Calcular a média

        for (byte x = 0 ; x < RATE_SIZE ; x++)
         beatAvg += rates[x];

        beatAvg /= RATE_SIZE;
      }
    }

    //Medir SpO2
    uint32_t ir, red ;
    double fred, fir;
    particleSensor.check(); //Verificar o sensor, ler até 3 amostras
    if (particleSensor.available()) {
      i++;
      ir = particleSensor.getFIFOIR(); //Ler o valor do infravermelho
      red = particleSensor.getFIFORed(); //Ler o valor da luz vermelha
      //Serial.println("red=" + String(red) + ",IR=" + String(ir) + ",i=" + String(i));
      fir = (double)ir; //Converter para double
      fred = (double)red; //Converter para double
      aveir = aveir * frate + (double)ir * (1.0 - frate); //Média do nível IR por filtro passa-baixa
      avered = avered * frate + (double)red * (1.0 - frate); //Média do nível vermelho por filtro passa-baixa
      sumirrms += (fir - aveir) * (fir - aveir); //Soma quadrada do componente alternado do nível IR
      sumredrms += (fred - avered) * (fred - avered); //Soma quadrada do componente alternado do nível vermelho

      if ((i % Num) == 0) {
        double R = (sqrt(sumirrms) / aveir) / (sqrt(sumredrms) / avered);
        //double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
        SpO2 = -23.3 * (R - 0.3) + 100;
        ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2; //Filtro passa-baixa
        if (ESpO2 <= MINIMUM_SPO2) ESpO2 = MINIMUM_SPO2; //Indicador de dedo removido
        //if (ESpO2 > 100) ESpO2 = 99.9;
        //Serial.print(",SPO2="); Serial.println(ESpO2);
        sumredrms = 0.0; sumirrms = 0.0; SpO2 = 0; i = 0;
      }
      particleSensor.nextSample(); //Finalizar esta amostra e passar para a próxima
    }
    
    //Exibir os dados no serial
    Serial.print("Bpm:" + String(beatAvg));
    //Exibir o valor de SpO2, para evitar medições incorretas, só exibir se a frequência cardíaca for maior que 30
    if (beatAvg > 30)  Serial.println(",SPO2:" + String(ESpO2));
    else Serial.println(",SPO2:" + String(ESpO2));

    //Exibir os dados no display OLED
    display.clearDisplay(); //Limpar a tela
    display.drawBitmap(5, 5, logo2_bmp, 24, 21, WHITE); //Exibir o ícone pequeno de coração
    display.setTextSize(2); //Definir o tamanho do texto
    display.setTextColor(WHITE); //Cor do texto
    display.setCursor(42, 10); //Definir a posição do cursor
    display.print(beatAvg); display.println(" BPM"); //Exibir o valor da frequência cardíaca
    display.drawBitmap(0, 35, O2_bmp, 32, 32, WHITE); //Exibir o ícone de oxigênio
    display.setCursor(42, 40); //Definir a posição do cursor
    //Exibir o valor de SpO2, para evitar medições incorretas, só exibir se a frequência cardíaca for maior que 30
    if (beatAvg > 30) display.print(String(ESpO2) + "%");
    else display.print("---- %" );
    display.display(); //Exibir na tela
  }
  //Se não detectar o dedo, limpar todos os dados e exibir "Coloque o dedo" na tela
  else {
    //Limpar os dados da frequência cardíaca
    for (byte rx = 0 ; rx < RATE_SIZE ; rx++) rates[rx] = 0;
    beatAvg = 0; rateSpot = 0; lastBeat = 0;
    //Limpar os dados de SpO2
    avered = 0; aveir = 0; sumirrms = 0; sumredrms = 0;
    SpO2 = 0; ESpO2 = 90.0;
    //Exibir "Coloque o dedo"
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(30, 5);
    display.println("Coloque");
    display.setCursor(30, 35);
    display.println("o dedo");
    display.display();
    //noTone(Tonepin);
  }
}