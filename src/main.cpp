#include "Arduino.h"

#define set_bit(y, bit) (y |= (1 << bit))  // coloca em 1 o bit x da variável Y
#define clr_bit(y, bit) (y &= ~(1 << bit)) // coloca em 0 o bit x da variável Y
#define cpl_bit(y, bit)                                                        \
  (y ^= (1 << bit)) // troca o estado lógico do bit x da variável Y
#define tst_bit(y, bit)                                                        \
  (y & (1 << bit)) // retorna 0 ou 1 conforme leitura do bit

#define DADOS_LCD PORTD // 4 bits de dados do LCD no PORTD
#define nibble_dados                                                           \
  1 // 0 para via de dados do LCD nos 4 LSBs do PORT empregado (Px0-D4, Px1-D5,
    // Px2-D6, Px3-D7)
// 1 para via de dados do LCD nos 4 MSBs do PORT empregado (Px4-D4, Px5-D5,
// Px6-D6, Px7-D7)
#define CONTR_LCD PORTB // PORT com os pinos de controle do LCD (pino R/W em 0).
#define E PB4           // pino de habilitação do LCD (enable)
#define RS PB3 // pino para informar se o dado é uma instrução ou caractere

// sinal de habilitação para o LCD
#define pulso_enable()                                                         \
  _delay_us(1);                                                                \
  set_bit(CONTR_LCD, E);                                                       \
  _delay_us(1);                                                                \
  clr_bit(CONTR_LCD, E);                                                       \
  _delay_us(45)

// protótipo das funções
void cmd_LCD(unsigned char c, char cd);
void inic_LCD_4bits();

//---------------------------------------------------------------------------------------------
// Sub-rotina para enviar caracteres e comandos ao LCD com via de dados de 4
// bits
//---------------------------------------------------------------------------------------------
void cmd_LCD(unsigned char c,
             char cd) // c é o dado  e cd indica se é instrução ou caractere
{
  if (cd == 0)
    clr_bit(CONTR_LCD, RS);
  else
    set_bit(CONTR_LCD, RS);

// primeiro nibble de dados - 4 MSB
#if (nibble_dados) // compila código para os pinos de dados do LCD nos 4 MSB do
                   // PORT
  DADOS_LCD = (DADOS_LCD & 0b00001111) | (0b11110000 & c);
#else // compila código para os pinos de dados do LCD nos 4 LSB do PORT
  DADOS_LCD = (DADOS_LCD & 0xF0) | (c >> 4);
#endif

  pulso_enable();

// segundo nibble de dados - 4 LSB
#if (nibble_dados) // compila código para os pinos de dados do LCD nos 4 MSB do
                   // PORT
  DADOS_LCD = (DADOS_LCD & 0b00001111) | (0b11110000 & (c << 4));
#else // compila código para os pinos de dados do LCD nos 4 LSB do PORT
  DADOS_LCD = (DADOS_LCD & 0xF0) | (0x0F & c);
#endif

  pulso_enable();

  if ((cd == 0) &&
      (c < 4)) // se for instrução de retorno ou limpeza espera LCD estar pronto
    _delay_ms(2);
}
//---------------------------------------------------------------------------------------------
// Sub-rotina para inicialização do LCD com via de dados de 4 bits
//---------------------------------------------------------------------------------------------
void inic_LCD_4bits() // sequência ditada pelo fabricando do circuito integrado
                      // HD44780
{                     // o LCD será só escrito. Então, R/W é sempre zero.

  clr_bit(CONTR_LCD,
          RS); // RS em zero indicando que o dado para o LCD será uma instrução
  clr_bit(CONTR_LCD, E); // pino de habilitação em zero

  _delay_ms(20); // tempo para estabilizar a tensão do LCD, após VCC
                 // ultrapassar 4.5 V (na prática pode ser maior).

  cmd_LCD(0x30, 0);

  pulso_enable(); // habilitação respeitando os tempos de resposta do LCD
  _delay_ms(5);
  pulso_enable();
  _delay_us(200);
  pulso_enable(); /*até aqui ainda é uma interface de 8 bits.
          Muitos programadores desprezam os comandos acima, respeitando apenas o
          tempo de estabilização da tensão (geralmente funciona). Se o LCD não
          for inicializado primeiro no modo de 8 bits, haverá problemas se o
          microcontrolador for inicializado e o display já o tiver sido.*/

  // interface de 4 bits, deve ser enviado duas vezes (a outra está abaixo)
  cmd_LCD(0x20, 0);

  pulso_enable();
  cmd_LCD(0x28, 0); // interface de 4 bits 2 linhas (aqui se habilita as 2
                    // linhas) são enviados os 2 nibbles (0x2 e 0x8)
  cmd_LCD(0x08, 0); // desliga o display
  cmd_LCD(0x01, 0); // limpa todo o display
  cmd_LCD(0x0F, 0); // mensagem aparente cursor inativo não piscando
  cmd_LCD(0x80,
          0); // inicializa cursor na primeira posição a esquerda - 1a linha
}

//--------------------------------------------------------
// codigo da pratica anterior a partir daqui

unsigned char d = 0;
unsigned long lastDispRefresh = 0, lastSerialRefresh = 0;
// Tabela = {0,1,2,3,4,5,6,7,8,9,E,r}
unsigned char Tabela[] = {0x40, 0x79, 0x24, 0x30, 0x19, 0x12,
                          0x02, 0x78, 0x80, 0x18, 0x06, 0x2F};
unsigned char todisp[3] = {10, 11, 11};

char ch = 0;

int sensorValue = 0;

void setup()
{
  // TODO: o que fazer para inicializar corretamento o LCD?

  DDRD |= 0b01111111;
  DDRB |= 0b00000111;

  DDRC &= ~(1 << PC0); // PC0 as input
  DDRC &= ~(1 << PC1); // PC1 as input

  ADMUX &= ~((1 << REFS0) | (1 << REFS1)); // tensao de referencia
  ADMUX |= (1 << REFS0);                   // AVCC

  ADCSRB = 0; // valor padrão

  ADCSRA &= 0b11111000;
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADCCLK = CLK/128

  // ADCSRA |= (1<<ADIE); // habilita interrupção
  ADCSRA |= (1 << ADEN); // habilita o ADC
}

void loop()
{
  // read the analog in value:
  // sensorValue = analogRead(A0);
  sensorValue = ler_adc(1);

  // na atividade 5 este trecho de código já foi deletado
  if (millis() > (lastSerialRefresh + 100))
  {
    lastSerialRefresh = millis();
    Serial.begin(115200);
    Serial.print(sensorValue);
    Serial.println();
    Serial.end();
  }

  // map it to the range of the analog out:
  int outputValue = map(sensorValue, 0, 1023, 0, 100);

  if (millis() > (lastDispRefresh + 0))
  {
    lastDispRefresh = millis();
    d++;
    d %= 3;

    PORTB =
        (PORTB & 0b11111000) | (1 << d); // ativa o display correspondente ao d
    PORTD = Tabela[todisp[d]];
  }

  // TODO: update the LCD from time to time (100ms?)

  delay(1); // Only for simulation
}

// busy waiting version
signed int ler_adc(unsigned char canal)
{
  ADMUX &= 0b11110000;
  ADMUX |= (0b00001111 & canal); // seleciona o canal ch no MUX
  DIDR0 = (1 << canal);

  ADCSRA |= (1 << ADSC); // inicia a conversão

  while (ADCSRA & (1 << ADSC))
    ; // espera a conversão ser finalizada

  return ADC;
}

// interrupt handler
ISR(ADC_vect)
{
  // TODO: pronto da atividade 5
}
