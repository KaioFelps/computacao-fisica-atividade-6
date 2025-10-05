#include "Arduino.h"
#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////////
// PINS ALIASES
////////////////////////////////////////////////////////////////////////////////////
/// 4 bits de dados do LCD no PORTD
#define DADOS_LCD PORTD

/**
 * Essa flag indica se os pinos que recebem dados no LCD são os 4 LSB⁽¹⁾ (Px0-D4
 * a Px3-D7) ou os 4 MSB⁽²⁾ (Px4-D4 a Px7-D7).
 *
 * ---
 *
 * 1: Less Significant Bits (bits menos significativos, isso é, os mais à
 * direita).
 * 2: Most Significant Bits (bits mais significativos, isso é, os mais
 * à esquerda).
 * Obs.: 1 nibble é meio byte (ou 4 bits).
 */
#define LCD_DATA_NIBBLE 1

/// PORT com os pinos de controle do LCD (pino R/W e Enable).
#define CONTR_LCD PORTB

/// Pino de habilitação do LCD (enable)
#define LCD_ENABLE PB4

/// Pino que informar se o dado é uma instrução ou caractere
/// - 0: indica que é uma instrução;
/// - 1: indica que é um caractere.
#define DATA_TYPE_PIN PB3

////////////////////////////////////////////////////////////////////////////////////
// MACROS
////////////////////////////////////////////////////////////////////////////////////
/// Coloca em 1 o `i`-nésimo bit da variável `y`.
#define set_bit(y, i) (y |= (1 << i))

/// Coloca em 0 o bit `i`-nésimo bit da variável `y`.
#define clr_bit(y, i) (y &= ~(1 << i))

/// Inverte o estado lógico do `i`-nésimo bit da variável `y`.
#define cpl_bit(y, i) (y ^= (1 << i))

/// Retorna 0 ou 1 conforme leitura do `i`-ésimo da variável `y`.
#define tst_bit(y, i) (y & (1 << i))

#if (LCD_DATA_NIBBLE == 1)
#define set_most_significant_nibble(data)                                      \
  DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & data);

#define set_less_significant_nibble(data)                                      \
  DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & (data << 4));
#else
#define set_most_significant_nibble(trimmed_data_nibble)                       \
  DADOS_LCD = (DADOS_LCD & 0xF0) | (data >> 4);

#define set_less_significant_nibble(data)                                      \
  DADOS_LCD = (DADOS_LCD & 0xF0) | (0x0F & data);
#endif

////////////////////////////////////////////////////////////////////////////////////
// ENUMS
////////////////////////////////////////////////////////////////////////////////////
enum class MessageType
{
  Instruction,
  Character
};

////////////////////////////////////////////////////////////////////////////////////
// Classes Declarations
////////////////////////////////////////////////////////////////////////////////////
class LcdFacade
{
public:
  /**
   * Envia caracteres ou comandos para o LCD utilizando a via de 4 bits (1
   * nibble).
   */
  static void send_message(uint8_t data, MessageType msg_type);

  /**
   * Envia um comando para o LCD. É o mesmo que
   * `LcdFacade::send_message(command_byte, MessageType::Instruction)`.
   */
  static void send_command(uint8_t command_byte);

  /**
   * Inicializa o LCD configurado para usar uma via de dados de 4 bits.
   */
  static void initialize_lcd();

private:
  /**
   * Coloca o pino `DATA_TYPE_PIN` no respectivo estado de acordo com o tipo de
   * dado da mensagem.
   */
  static void set_message_data_type_to_pin(MessageType msg_type);

  /**
   * Coloca os dados (`data`) no LCD nibble-a-nibble.
   */
  static void set_data_to_lcd(uint8_t data);

  /**
   * Manda um sinal de habilitação (pulso) para o LCD.
   */
  static void enable_pulse();
};

////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS DECLARATIONS
////////////////////////////////////////////////////////////////////////////////////

void LcdFacade::enable_pulse()
{
  _delay_us(1);
  set_bit(CONTR_LCD, LCD_ENABLE);
  _delay_us(1);
  clr_bit(CONTR_LCD, LCD_ENABLE);
  _delay_us(45);
}

void LcdFacade::set_data_to_lcd(uint8_t data)
{
  set_most_significant_nibble(data);
  set_less_significant_nibble(data);
}

void LcdFacade::set_message_data_type_to_pin(MessageType msg_type)
{
  switch (msg_type)
  {
  case MessageType::Instruction:
    clr_bit(CONTR_LCD, DATA_TYPE_PIN);
    break;
  case MessageType::Character:
    set_bit(CONTR_LCD, DATA_TYPE_PIN);
    break;
  }
}

void LcdFacade::send_message(uint8_t data, MessageType msg_type)
{
  set_message_data_type_to_pin(msg_type);
  set_data_to_lcd(data);

  enable_pulse();

  const auto is_return_or_clean_instruction =
      msg_type == MessageType::Instruction && data < 4;

  if (is_return_or_clean_instruction) _delay_ms(2);
}

void LcdFacade::send_command(uint8_t command_byte)
{
  send_message(command_byte, MessageType::Instruction);
}

void LcdFacade::initialize_lcd()
{
  // Essa sequência é ditada pelo fabricante do circuito integrado HD44780.
  // O LCD será só escrito, então, R/W deve ser sempre zero.

  // Indica que é uma instrução
  clr_bit(CONTR_LCD, DATA_TYPE_PIN);
  // Manualmente desabilita o LCD
  clr_bit(CONTR_LCD, LCD_ENABLE);

  // Tempo necessário para estabilizar a tensão do LCD (após o VCC
  // ultrapassar 4.5V — que pode ser maior na prática).
  _delay_ms(20);

  send_message(0x30, MessageType::Instruction);

  // Liga o LCD respeitando o tempo de resposta do próprio LCD.
  enable_pulse();
  _delay_ms(5);
  enable_pulse();
  _delay_ms(200);
  enable_pulse();
  // Até aqui ainda é uma interface de 8 bits.
  // Muitos programadores desprezam os comandos acima, respeitando apenas
  // o tempo de estabilização da tensão (geralmente funciona). Se o LCD
  // não for inicializado primeiro no modo de 8 bits, haverá problemas se
  // o microcontrolador for inicializado e o display já o tiver sido.

  // Necessário para forçar a interface de 4 bits, deve ser enviado duas vezes
  // (a outra está abaixo).
  send_command(0x20);

  // Essa instrução vai:
  // * Finaliza a configuração para usar interface de 4 bits;
  // * Configurar para usar 2 linhas da memória RAM;
  // * Configurar a fonte de caracteres para uma proporção 5 ⨉ 8.
  enable_pulse();
  send_command(0x28);

  // Desliga o display
  send_command(0x08);
  // Limpa todo o display
  send_command(0x01);
  // Mostra a mensagem no display, mas desativa o cursor
  send_command(0x0F);
  // Inicializa o cursor na primeira posição à esquerda (1ᵃ linha).
  send_command(0x80);
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
