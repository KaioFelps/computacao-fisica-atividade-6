#include "Arduino.h"
#include <stdint.h>

void setup()
{
  // TODO: o que fazer para inicializar corretamento o LCD?

  // Torna os primeiros 7 pinos do registrador D saídas (são os segmentos do
  // led).
  DDRD |= 0b01111111;

  // Torna os 3 primeiros pinos do registrador B em saídas (são os pinos dos
  // leds).
  DDRB |= 0b00000111;

  // Torna os bits PC0 e PC1 do registrador DDRC inputs (são as entradas dos
  // potenciômetros), apesar de que já são entradas por padrão.
  DDRC &= ~(1 << PC0);
  DDRC &= ~(1 << PC1);

  // Configuração do conversor analógico-digital do microcontrolador do
  // Arduino (ATmega328).
  //
  // Esse trecho coloca o ADMUX num estado 01 (REF0:REF1) que diz que a tensão
  // usada pela porta AVCC (que alimenta o circuito analógico do
  // microcontrolador) será a mesma tensão usada como a tensão de referência
  // (valor máximo que o ADC pode medir).
  ADMUX &= ~((1 << REFS0) | (1 << REFS1));
  ADMUX |= (1 << REFS0);

  // * ADCSRx: ADC Control and Status Register x
  // * ADPSx: ADC Prescaler Select Bit x (é o x-nésimo bit menos significativo
  // do registrador ADCSRx)

  // Zera os bits do controlador & status B.
  // Esse registrador controla a fonte de gatilho, entrada e modo do ADC. Ao
  // zerar, estamos:
  // * desabilitando a inicialização automática do ADC
  // * deixando o ADC funcionar no seu modo simples de medição
  // * desligamos outras configurações avançadas do ADC
  ADCSRB = 0;

  // Configura individualmente os 3 primeiros bits do controlador & status A,
  // que definem o clock do conversor ADC (e outras funcionalidades do ADC).
  ADCSRA &= 0b11111000;
  // Essa configuração (0b111 = 128) divide o clock principal do
  // microcontrolador por 128 para formar o clock de trabalho do ADC, isso é,
  // declara que ADDCLK = CLK / 128.
  //
  // Para obter a precisão máxima de um ADC de 10 bits, o clock de conversão
  // deve estar entre 50kHz e 200kHz, e com esse cálculo, conseguimos chegar num
  // valor próximo, pois o arduino uno tem um clock de 16MHz, e 16000 (16 mega
  // hertz) / 128 = 125 (em kiloheartz).
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  // ADIE: ADC Interrupt Enable
  // Permite que uma interrupção (`ADC_vect) seja acionada sempre que o ADC
  // terminar de medir a tensão e tiver um valor pronto.
  ADCSRA |= (1 << ADIE);

  // ADEN: ADC Enable
  // Liga o ADC manualmente (a nível de hardware, tirando essa parte do circuito
  // do modo de baixo consumo de energia).
  ADCSRA |= (1 << ADEN);

  // DIDRx: Digital Input Disable Register x
  // Um único pino no ATmega328 pode ser uma saída ou uma entrada (digital ou
  // analógica); ele é multifuncional.
  //
  // A funcionalidade de entrada digital (um amplificador conectado no pino pra
  // realizar a leitura digital) pode criar um ruído que atrapalha a leitura da
  // entrada analógica. Por isso, desligamos o circuito que faz a leitura
  // digital no pino que será lido para evitar que ele gere ruído e atrapalhe a
  // leitura analógica.
  //
  // Observe que:
  // Quando um bit em DIDRx é:
  // * 0, então o buffer digital está ligado
  // * 1, então o buffer digital está desligado
  //
  // Aqui, desligamos o buffer digital de todos os pinos da porta C (de inputs
  // analógicos). Podemos desligar todos pois somente os 2 primeiros pinos estão
  // sendo utilizados, e são os do potenciômetro. Portanto, não afetamos nenhum
  // outro periférico. do pino em que o potenciômetro selecionado está conectado
  DIDR0 = 0b00111111;

  // Inicia a primeira conversão. As posteriores serão iniciadas ao final de
  // cada interrupção.
  ADCSRA |= (1 << ADSC);
}

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

/**
 * Pino que informar se o dado é uma instrução ou caractere
 * - 0: indica que é uma instrução;
 * - 1: indica que é um caractere.
 */
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

enum class AvailablePotentiometer
{
  Left = 0,
  Right = 1
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

typedef struct
{
  uint16_t left_potentiometer_acc;
  uint16_t right_potentiometer_acc;
  uint16_t mean;
  bool hasError;
} Acceleration;

class SSDisplayFacade
{

public:
  static void print_output_value(uint8_t output_value, uint8_t active_digit);
  static void print_err_msg(uint8_t active_digit);

private:
  static void print_char_on_digit_led(const uint8_t character,
                                      const uint8_t digit);
};

class PotentiometersFacade
{
public:
  static Acceleration parse_potentiometers_acceleration(uint16_t left,
                                                        uint16_t right);

  static void update_potentiometers_read_states(
      AvailablePotentiometer *active_read_potentiometer,
      uint16_t *left_potentiometer, uint16_t *right_potentiometer,
      bool *should_parse_potentiometers);
};

////////////////////////////////////////////////////////////////////////////////////
// GLOBAL STATIC CONSTANTS
////////////////////////////////////////////////////////////////////////////////////
/*
 * @brief Representa qual dígito vai ser ativado na corrente iteração do`loop`.
 */
uint8_t active_digit = 0;

/// @brief Contém a representação dos caracteres em segmentos do display.
uint8_t SEGMENTS_CHARS[] = {
    0x40, // 0
    0x79, // 1
    0x24, // 2
    0x30, // 3
    0x19, // 4
    0x12, // 5
    0x02, // 6
    0x78, // 7
    0x80, // 8
    0x18, // 9
    0x06, // E
    0x2F  // r
};

////////////////////////////////////////////////////////////////////////////////////
// LOOP & ARDUINO INTERRUPTIONS
////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // TODO: update the LCD from time to time (100ms?)
}

// Código a ser executado sempre que o ADC terminar a conversão de um sinal
// analógico
ISR(ADC_vect)
{
  static uint16_t left_potentiometer_stable_value = 0;
  static uint16_t right_potentiometer_stable_value = 0;
  static uint8_t active_digit = 0;
  static Acceleration acceleration = {
      .left_potentiometer_acc = 0,
      .right_potentiometer_acc = 0,
      .mean = 0,
      .hasError = false,
  };

  const uint16_t display_multiplex_rate = 10;
  /*
   * @brief A tensão de referência configurada no `setup` faz com que esta
   * interrupção seja executada 1x a cada 104 microssegundos. Pela fórmula da
   * frequência (F = 1/T), a frequência desta ISR é 1/104*1e-6 = 1/0,000104 ≈
   * 9,6Khz ≈ 10Khz. Para multiplexar os displays numa frequência de 1Khz,
   * precisamos que esse contador (de frequência) seja incrementado até 10 para
   * sinalizar a troca dos displays.
   *
   * Observe que isso garante que o código ALTERNE PARA O PRÓXIMO DISPLAY na
   * frequência aproximada de 1Khz, e não que os três displays sejam acionados
   * nesta frequência. Como a explicação ficou um pouco ambígua, optei por esta
   * alternativa.
   */
  static uint16_t freq_counter = 0;

  static AvailablePotentiometer active_read_potentiometer =
      AvailablePotentiometer::Left;

  auto should_parse_potentiometers_values = false;

  PotentiometersFacade::update_potentiometers_read_states(
      &active_read_potentiometer, &left_potentiometer_stable_value,
      &right_potentiometer_stable_value, &should_parse_potentiometers_values);

  if (should_parse_potentiometers_values)
  {
    should_parse_potentiometers_values = false;
    acceleration = PotentiometersFacade::parse_potentiometers_acceleration(
        left_potentiometer_stable_value, right_potentiometer_stable_value);
  }

  if (freq_counter >= display_multiplex_rate)
  {
    freq_counter = 0;

    // Multiplexa o dígito ativo.
    active_digit++;
    active_digit %= 3;

    if (acceleration.hasError)
    {
      SSDisplayFacade::print_err_msg(active_digit);
    }
    else
    {
      SSDisplayFacade::print_output_value(acceleration.mean, active_digit);
    }
  }
  else
  {
    freq_counter++;
  }

  // Inicia a próxima conversão.
  ADCSRA |= (1 << ADSC);
}

////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS DEFINITIONS & IMPLEMENTATIONS
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

void SSDisplayFacade::print_err_msg(uint8_t active_digit)
{
  const uint8_t err_msg[3] = {
      (uint8_t)10,
      (uint8_t)11,
      (uint8_t)11,
  };

  print_char_on_digit_led(err_msg[active_digit], active_digit);
}

void SSDisplayFacade::print_output_value(uint8_t output_value,
                                         uint8_t active_digit)
{
  const uint8_t output_value_digits[3] = {
      (uint8_t)(output_value / 100),
      (uint8_t)(output_value / 10 % 10),
      (uint8_t)(output_value % 10),
  };

  const auto digit_to_show_currently = output_value_digits[active_digit];
  print_char_on_digit_led(digit_to_show_currently, active_digit);
}

void SSDisplayFacade::print_char_on_digit_led(const uint8_t character,
                                              const uint8_t digit)
{
  PORTB &= 0b11111000;
  PORTD = SEGMENTS_CHARS[character];
  PORTB |= (1 << digit);
}

Acceleration
PotentiometersFacade::parse_potentiometers_acceleration(uint16_t left,
                                                        uint16_t right)
{
  const uint16_t adc_min_read_value = 0;
  const uint16_t adc_max_read_value = 1023;

  Acceleration acceleration = {
      .left_potentiometer_acc = left,
      .right_potentiometer_acc = right,
  };

  acceleration.mean = map((left + right) / 2, 138, 623, 0, 100);

  // Divisão por 2 melhora comparações para cálculo de 10%
  uint16_t left_acc = (acceleration.left_potentiometer_acc) / 2;
  uint16_t right_acc = acceleration.right_potentiometer_acc;

  // Uma das formas de buscar a porcentagem de uma diferença é:
  // Calcular a diferença entre os valores desejados
  // Multiplicar por 100 ( porcentagem )
  // Dividir pelo menor
  uint16_t smaller = (left_acc < right_acc) ? left_acc : right_acc;
  uint16_t diff = abs((int)left_acc - (int)right_acc);

  // A verificação do smaller ser menor que 10 serve para evitar
  // a divisão por 0 ou valores muito pequenos.
  if (smaller < 10 || acceleration.mean > 100 || ((diff * 100) / smaller) > 10)
  {
    acceleration.hasError = true;
  }
  else
  {
    acceleration.hasError = false;
  }

  return acceleration;
}

void PotentiometersFacade::update_potentiometers_read_states(
    AvailablePotentiometer *active_read_potentiometer,
    uint16_t *left_potentiometer, uint16_t *right_potentiometer,
    bool *should_parse_potentiometers)
{
  const auto new_adc_value = ADC;
  switch (*active_read_potentiometer)
  {
  case AvailablePotentiometer::Left:
    *left_potentiometer = new_adc_value;
    *active_read_potentiometer = AvailablePotentiometer::Right;
    break;
  case AvailablePotentiometer::Right:
    *right_potentiometer = new_adc_value;
    *active_read_potentiometer = AvailablePotentiometer::Left;
    *should_parse_potentiometers = true;
    break;
  }

  const auto potentiometer_pin =
      static_cast<uint8_t>(*active_read_potentiometer);

  // Seleciona a leitura do potenciômetro escolhido no multiplexer de leituras
  // analógicas do ADC.
  ADMUX &= 0b11110000;
  ADMUX |= (0b0000111 & potentiometer_pin);
}
