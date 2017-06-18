/*
 * mini Wave_Gen - Arduino (nano/uno) based function generator
 * - using DAC MCP4725, 12bit, i2c
 * - from DC to 568 Hz (limited by arduino nano hardware)
 * Control of:
 * - samples (16, 32, 64, 128 sps), frequency, amplitude, offset (dc level)
 * 
 * Use "micros( )" function to control the temporization between samples
 * - TIMER1 interrupt is not compatible with DAC library
 * ----------------------------------------------------------------------------
 * mini Wave_Gen - Gerador de funções baseado no Arduino Nano/Uno
 * - Utiliza um DAC MCP4725 de 12 bit via i2c
 * - Sinais de DC até 568 Hz (limitação do Arduino Nano)
 * Controle de:
 * - Amostras (16, 32, 64 ou 128), Frequência, Amplitude e Offset (DC level)
 * 
 * Utiliza função "micros( )" para temporização entre amostras
 * - interrupção do TIMER1 não foi compatível com a lib do DAC (melhoria futura)
 * * ----------------------------------------------------------------------------
 * o artigo sobre este código esta disponível em:
 * http://blog.filipeflop.com/author/haroldo-amaral/
 * https://github.com/agaelema/mini-Wave_Gen
 * 
 * developed by: Haroldo Amaral - agaelema@gmail.com
 * 2017/06/05 - v 1.0
 */

#include <Wire.h>
#include <Adafruit_MCP4725.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

/*
 * constantes importantes - important constants
 * - Vref:  voltage reference value, theoretically 5V. Measured with multimeter
 *          tensão de referência do DAC (VCC) - valor medido com um multímetro - 5V (ideal)
 * - time_compensation: compensation of calculated frequency
 *                      compensação da freq calculada - valor empírico de 0.985
 * - SW1/SW2: Digital pins (switches), configuration and value selection
 *            pinos digitais utilizados para chaves, configuração e seleção de valores
 */
const PROGMEM float Vref = 4.975;
const PROGMEM float time_compensation = 0.985;
const uint8_t SW1 = 2;
const uint8_t SW2 = 3;

/*
 * buffer_size:   maximum number of samples // número máximo de amostras
 * sample_size:   possible samples // amostragem disponível
 */
const unsigned int buffer_size = 128;
const uint8_t sample_size[] = {16, 32, 64, 128};

/*
 *  Define here the initial values - Defina aqui os valores iniciais
 * - freq, amplitude, offset, samples
 */
const PROGMEM float initial_freq = 50;
const PROGMEM float initial_amplitude = Vref;
const PROGMEM float initial_offset = Vref/2;
const PROGMEM float initial_samples = 32;     // 16, 32, 64, 128

/*
 * Rotary encoder variables - variáveis associadas ao encoder
 * - pins 6 e 7 (INPUT)
 */
const uint8_t Encoder_CLK = 6;
const uint8_t Encoder_DATA = 7;
uint32_t currentTime;
uint32_t loopTime;
uint8_t encoder_A;
uint8_t encoder_B;
uint8_t encoder_A_prev = 0;
int8_t enc_value;

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

Adafruit_MCP4725 dac;     //creates an instance of DAC / cria instância do DAC MCP4725

/*
 * stores the state of switches - armazena estados relacionados as chaves pressionadas
 * - SW1: RUNNING, CONFIGURING, CALCULATING...
 * - SW2: contagem relativa aos campos modificáveis
 */
uint8_t SW1_state = 0;
uint8_t SW2_state = 0;

// enumera os possíveis estados
enum
{
  RUNNING,        // executando a onda
  CONFIGURING,    // configurando os parâmetros
  CALCULATING     // calculando nova forma de onda
};

uint16_t i = 0;       // variável auxiliar para contagem

/*
 * armazenam contagem da função micros() - uS passados
 */
uint32_t last_sample_time;
uint32_t last_sample_time2;

/*
 * wave related variables // variables Variáveis relacionadas a onda
 * - wave[buffer_size] - buffer with the calculated wavefomr // buffer que armazena a forma de onda calculada
 * - wave_parameters[4] - array with waveform parameter //array que armazena valores dos parâmetros do usuário
 * - sample_interval - sample interval (in microseconds)// intervalo entre cada amostra (em microsegundos)
 */
uint16_t wave[buffer_size];
float wave_parameters[4];
uint32_t sample_interval = 0;
const uint16_t freq_max = 568;

//uint16_t freq_uint = 0;
uint16_t samples_uint = 0;

// which parameters can be configured // enumera quais itens podem ser configurados
enum{ FREQ, AMPLITUDE, OFFSET, SAMPLES };

// texto de cada forma de onda no oled
const char string_sine[] PROGMEM =    "SINE  ";
const char string_ramp[] PROGMEM =    "RAMP  ";
const char string_square[] PROGMEM =  "SQUARE";
const char string_dc[] PROGMEM =      "DC    ";

// cria um array de strings
const char* const wave_text_1[] PROGMEM = {string_sine,string_ramp, string_square, string_dc};

// texto adicional de cada forma de onda no oled
const char string_freq_0[] PROGMEM =    "Freq: ";
const char string_freq_1[] PROGMEM =    " Hz  ";
const char string_amp_0[] PROGMEM =     "Amp : ";
const char string_amp_1[] PROGMEM =     " V   ";
const char string_offset_0[] PROGMEM =  "Voff: ";
const char string_offset_1[] PROGMEM =  " V   ";
const char string_sample_0[] PROGMEM =  "Samp: ";
const char string_sample_1[] PROGMEM =  "     ";

// array de strings - textos adicionais
const char* const parameter_text[] PROGMEM =
{
  string_freq_0, string_freq_1,
  string_amp_0, string_amp_1,
  string_offset_0, string_offset_1,
  string_sample_0, string_sample_1
};

// waveforms available // enumera as formas de onda disponíveis
enum{ SINE, RAMP, SQUARE, DC };

// display update type // tipos de atualização do display
enum{ DISPLAY_VALUES, DISPLAY_ALL };

/*
 * multiplier aplied to each sub-parameter // multiplicadores utilizados para cada edicao de valor
 * - multiplied by the encoder value (0, 1, -1) // multiplica pelo valor do encoder (0, 1, -1)
 */
const float multipliers[] PROGMEM
{
  1,     //0 - waveform
  10,    //1 - freq
  1,     //2 - freq
  1,     //3 - amp
  0.1,   //4 - amp
  0.01,  //5 - amp
  1,     //6 - off
  0.1,   //7 - off
  0.01,  //8 - off
  1      //9 - samples
};

float float_pgm_read = 0;

// select the default waveform // seleciona a forma de onda padrão
uint8_t waveform = SINE; 

/*
 * Prototype of functions // Protótipo das funções (ao final)
 */
void wave_calc(uint16_t *array, uint16_t points, float amplitude, float offset, uint16_t wave);
int8_t read_Encoder();
void update_freq();
void update_display(uint8_t select);
void splash_display();
void SW1_interrupt();
void SW2_interrupt();

/*
 * Configure peripherals
 * realiza configuração dos periféricos
 */
void setup(void)
{
  // setup serial port
  Serial.begin(9600);

  // iniciar o dac no endereço 0x63
  dac.begin(0x63);

  // inicia o display oled no endereço 0x3c - 128x64
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  
  /*
   * Welcome message - serial and display
   * Mensagem de boas vindas - serial e display
   */
  Serial.println( F("mini Wave_Gen") );
  Serial.println( F("by: Haroldo Amaral - agaelema@gmail.com") );

  // limpa o buffer e atualiza o display (vazio)
  display.clearDisplay();
  splash_display();

  /*
   * enable interrupt on pins 6 and 7 // habilita interrupção nos pinos 6 e 7
   * call the functios "SW1_interrupt" and "SW2_interrupt // chama as funções "SW1_interrupt" e "SW2_interrupt
   * - increment the flags // apenas incrementam flags
   */
  pinMode(SW1, INPUT_PULLUP);
  attachInterrupt( digitalPinToInterrupt(SW1), SW1_interrupt, FALLING );
  pinMode(SW2, INPUT_PULLUP);
  attachInterrupt( digitalPinToInterrupt(SW2), SW2_interrupt, FALLING );
  
  // onboard led // led onboard (pino 13) como sinalizador - saída
  pinMode (LED_BUILTIN, OUTPUT);

  /*
   * Load the initial setup // Carrega as configurações iniciais
   * - first loaded waveform // primeira forma de onda a ser carregada
   */
  wave_parameters[FREQ] = pgm_read_float(&initial_freq);
  wave_parameters[AMPLITUDE] = pgm_read_float(&initial_amplitude);
  wave_parameters[OFFSET] = pgm_read_float(&initial_offset);
  wave_parameters[SAMPLES] = pgm_read_float(&initial_samples);

  /*
   * define the initial state // define estado inicial para as variáveis
   * - starts calculating the waveform // inicia calculando a forma de onda definida
   */
  SW1_state = CALCULATING;
  SW2_state = 0;

  // limpa o buffer e atualiza o display (vazio)
  display.clearDisplay();
  display.display();

  // atualiza o display inteiro
  update_display(DISPLAY_ALL);

  // salva a contagem de tempo atual
  last_sample_time = micros();
  last_sample_time2 = micros();

  /*
   * Configure encoder pins // Configura os pinos do encoder como entrada
   * - save the current time // armazena o instante atual em ms
   */
  pinMode(Encoder_CLK, INPUT);
  pinMode(Encoder_DATA, INPUT);
  currentTime = millis();
  loopTime = currentTime;

  // registra o estado inicial do encoder
  encoder_A_prev = digitalRead(Encoder_CLK);
}

void loop(void)
{
  /*
   * if the state variable is equal to 0 - runs the waveform
   * - still in this state while SW is not pressed
   * if the state variable is equal to 1 - configures the waveform
   * - take the user values
   * - pressing SW will lead to calculating state
   * if the state variable is equal to 2 - calculates the waveform samples
   * - return to 0 state (running)
   * --------------------------------------------------------------------------
   * se a variável de estado estiver em 0 - reproduz a forma de onda
   * - fica neste estado enquanto a chave SW1 não for pressionada
   * Se a variável de estado estiver em 1 - configura a forma de onda
   * - recebe valores do usuário
   * - pressionado SW1 sai deste "if" e entra no cálculo
   * se a variável de estado estiver em 2 - calcula a nova forma de onda
   * - retorna ao estado 0 para execução
   */
  if (SW1_state == RUNNING)
  {
    /*
     * runs the waveform stored in the buffer
     * - Use "micros ()" as reference because the DAC drive interferes with the i2c
     * - is not accurate as use a hw timer, but works
     * ------------------------------------------------------------------------------
     * executa a forma de onda calculada e armazenada no buffer
     * - utilizar como referência a função "micros( )" pois
     *   modificações no TIMER1 afetam a i2c (display e DAC)
     * - Não é tão preciso quanto um timer, mas funcionou bem
     */
    if ( (micros() - last_sample_time) >= sample_interval )
    {
      last_sample_time = micros();
      dac.setVoltage(wave[i & (samples_uint - 1)], false);
      i++;
    }
    else;
  }
  
  else if (SW1_state == CONFIGURING)
  {
    /*
     * Update oled screen every 500ms // realiza a atualização do display a cada 500ms
     * - minimize overload //minimizar processamento
     * - does not interfere with encoder // não interferir na leitura do encoder
     */
    if ( (micros() - last_sample_time2) >= 500000 )
    {
      /*
       * saturates variables when necessary // satura as variáveis quando necessário
       * - antes de atualizar o display, verificar se não ultrapassaram os limites
       */
      if ( (uint16_t)wave_parameters[FREQ] > freq_max / ( (uint16_t)wave_parameters[SAMPLES]>>4) )
      {
        wave_parameters[FREQ] = (float)freq_max / ( (uint16_t)wave_parameters[SAMPLES]>>4 );
      }
      else if (wave_parameters[FREQ] < 0)
      {
        wave_parameters[FREQ] = (float)1.0;
      }
      else;
      
      if (wave_parameters[AMPLITUDE] > Vref)
      {
        wave_parameters[AMPLITUDE] = Vref;
      }
      else if (wave_parameters[AMPLITUDE] < 0)
      {
        wave_parameters[AMPLITUDE] = 0;
      }
      else;
      
      if (wave_parameters[OFFSET] > Vref)
      {
        wave_parameters[OFFSET] = Vref;
      }
      else if (wave_parameters[OFFSET] < 0)
      {
        wave_parameters[OFFSET] = 0;
      }
      else;

      if (wave_parameters[SAMPLES] < 16)
      {
        wave_parameters[SAMPLES] = 16;
      }
      else;

      // atualiza os valores no display
      update_display(DISPLAY_VALUES);
      
      last_sample_time2 = micros();
    }

    uint16_t aux = 0;
    
    // le o encoder
    enc_value = read_Encoder();

    // inverte o led - pinsca indicando configuração
    digitalWrite(13, digitalRead(13)^1);

    /*
     * For each state fo SW (values between 0 and 9)
     * - enable to manipulate the wave parameters
     * Read the value returned by the encoder function
     * ---------------------------------------------------
     * para cada estado em SW2 (valores entre 0 e 9)
     * - permite manipular uma das variáveis do usuário
     * Lê o valor retornado pelo encoder (0, 1, -1)
     * - aplica um multiplicador específico
     */
    switch(SW2_state)
    {
      // caso 0 - manipula qual forma de onda
      case 0:
        float_pgm_read = pgm_read_float(multipliers + 0);
        waveform = ( waveform + (uint8_t)float_pgm_read*enc_value ) & 0x03;   //limita a 4 formas de onda
        break;

      // caso 1 - manipula algarismo da dezena da frequencia - Yx.xx
      case 1:
        float_pgm_read = pgm_read_float(multipliers + 1);
        wave_parameters[FREQ] += (float)float_pgm_read*enc_value;
        // if "dc" jumps to 6 //caso a onda for "DC" pula para o case 6 indiretamente
        if (waveform == DC)
        {
          SW2_state = 6;
        }else;
        break;

      // caso 2 - manipula algarismo da unidade da frequencia - xY.xx
      case 2:
        float_pgm_read = pgm_read_float(multipliers + 2);
        wave_parameters[FREQ] += (float)float_pgm_read*enc_value;
        break;

      // caso 3 - manipula algarismo da unidade da amplitude -  Y.xx
      case 3:
        float_pgm_read = pgm_read_float(multipliers + 3);
        wave_parameters[AMPLITUDE] += (float)float_pgm_read*enc_value;
        break;

      // caso 4 - manipula algarismo do décimo da amplitude -  x.Yx
      case 4:
        float_pgm_read = pgm_read_float(multipliers + 4);
        wave_parameters[AMPLITUDE] += (float)float_pgm_read*enc_value;
        break;

      // caso 5 - manipula algarismo do centésimo da amplitude -  x.xY
      case 5:
        float_pgm_read = pgm_read_float(multipliers + 5);
        wave_parameters[AMPLITUDE] += (float)float_pgm_read*enc_value;
        break;

      // caso 6 - manipula algarismo da unidade do offset -  Y.xx
      case 6:
        float_pgm_read = pgm_read_float(multipliers + 6);
        wave_parameters[OFFSET] += (float)float_pgm_read*enc_value;
        break;

      // caso 7 - manipula algarismo do décimo do offset -  x.Yx
      case 7:
        float_pgm_read = pgm_read_float(multipliers + 7);
        wave_parameters[OFFSET] += (float)float_pgm_read*enc_value;
        break;

      // caso 8 - manipula algarismo do centésimo do offset -  x.xY
      case 8:
        float_pgm_read = pgm_read_float(multipliers + 8);
        wave_parameters[OFFSET] += (float)float_pgm_read*enc_value;
        break;

      // caso 9 - Manipula a quantidade de amostras
      case 9:
        float_pgm_read = pgm_read_float(multipliers + 9);
        aux = (uint16_t)wave_parameters[SAMPLES];
        if (enc_value == 1)
        {
          aux <<= 1;
        }
        else if (enc_value == -1)
        {
          aux >>= 1;
        }
        else;

        aux = aux & 0xF0;           // restringe os resultados entre 16 e 128 amostras
        wave_parameters[SAMPLES] = (float)aux;
        break;

      default:
        SW2_state = 0;
    }
  }
  
  else if (SW1_state == CALCULATING)
  {
    samples_uint = (uint16_t)wave_parameters[SAMPLES];

    if (waveform == SINE)
    {
      wave_calc(wave, samples_uint, wave_parameters[AMPLITUDE], wave_parameters[OFFSET], waveform);
    }
    else if (waveform == RAMP)
    {
      wave_calc(wave, samples_uint, wave_parameters[AMPLITUDE], wave_parameters[OFFSET], waveform);
    }
    else if (waveform == SQUARE)
    {
      wave_calc(wave, samples_uint, wave_parameters[AMPLITUDE], wave_parameters[OFFSET], waveform);
    }
    else if (waveform == DC)
    {
      wave_calc(wave, samples_uint, wave_parameters[AMPLITUDE], wave_parameters[OFFSET], waveform);
    }
    else;

    update_freq();

    // atualiza dos estados      
    SW1_state = RUNNING;
    SW2_state = 0;
  }
  else
  {
    SW1_state = 0;
  }
}

/***********************************************************************************
 *                              Funções - Functions
 ***********************************************************************************/

/*
 * calculates each sample value // Calcula o valor de cada amostra para a onda selecionada
 * Arguments //Argumentos:
 *  - *array    : array com as amostras
 *  - points    : número de amostras
 *  - amplitude : amplitude do sinal (Vpp)
 *  - offset    : tensão de offset (Vdc)
 *  - wave      : qual forma de onda
 */
void wave_calc(uint16_t *array, uint16_t points, float amplitude, float offset, uint16_t wave)
{
  volatile uint16_t i;
  float temp_float;
  int32_t temp = 0;

  Serial.println();
  Serial.print(F("Calculando "));

  float v_const = 0;
  float const_sin_rad = 0;
  float level_variation = 0;

  if (wave == SINE)
  {
    Serial.println(F("Senoide"));
    v_const = (4096 / Vref) * amplitude / 2;
    const_sin_rad = (float)(2 * PI) / (points);
  }
  else if (wave == RAMP)
  {
    Serial.println(F("Rampa"));
    v_const = (4096 / Vref) * amplitude / 1;
    level_variation = (float)1/points;
  }
  else if (wave == SQUARE)
  {
    Serial.println(F("Quadrada"));
    v_const = (4096 / Vref) * amplitude / 1;
  }
  else if (wave == DC)
  {
    Serial.println(F("DC level"));
    v_const = (4096 / Vref) * amplitude / 1;
  }
  else;
  
  /*
   * calcula o valor de cada amostra
   * - cada forma de onda exite um cálculo diferente
   * - todos os cálculos retornam o valor pela variável "temp"
   */
  for (i = 0; i < points; i++)
  {
    if (wave == SINE)
    {
      temp_float = (sin(const_sin_rad * i * 1));
      temp_float = (temp_float * v_const) + (float)(4096 / Vref) * offset;
      temp = (int32_t)temp_float;
    }
    else if (wave == RAMP)
    {
      temp_float = i * level_variation;
      temp_float = (temp_float * v_const) + (float)(4096 / Vref) * offset;
      temp = (int32_t)temp_float;
    }
    else if (wave == SQUARE)
    {
      if (i < points/2)
      {
        temp_float = v_const + (float)(4096 / Vref) * offset;
        temp = (int32_t)temp_float;
      }
      else
      {
        temp_float = (float)(4096 / Vref) * offset;
        temp = (int32_t)temp_float;
      }
    }
    else if (wave == DC)
    {
      temp_float = (float)(4096 / Vref) * offset;
      temp = (int32_t)temp_float;
    }
    else;

    if (temp > 4095)
    {
      array[i] = 4095;
    }
    else if (temp < 0)
    {
      array[i] = 0;
    }
    else
    {
      array[i] = temp;
    }

    // imprime o valor calculado
    Serial.println(array[i]);
  }
}

/*
 * realiza a leitura do encoder
 * - retorna
 *   0 - nenhuma mudança
 *   1 - incremento
 *  -1 - decremento
 */
int8_t read_Encoder()
{
  int8_t value = 0;
  encoder_A = digitalRead(Encoder_CLK);
  encoder_B = digitalRead(Encoder_DATA);

  // se ocorreu uma transição de "1" para "0"
  if((!encoder_A) && (encoder_A_prev))
  {
    // Se encoder_B == 1
    if(encoder_B)
    {
      value++;
    }
    // Se encoder_B = 0
    else
    {
      value--;
    }   
  }
  encoder_A_prev = encoder_A;     // salva o valor atual
  return value;
}

/*
 * calcula o intervalo entre as amostras em microsegundos
 *  - intervalo = perído do sinal / quantidade de amostras
 */
void update_freq()
{
  volatile float time_temp;
  time_temp = (float)1 / wave_parameters[FREQ];
  time_temp = time_temp / wave_parameters[SAMPLES];
  time_temp = time_temp * 1000000;
  time_temp = time_temp * time_compensation;
  sample_interval = (uint32_t)time_temp;
}

/*
 * Função que atualiza o display - de acordo com o argumento
 * - 0: parcial - valores e nome da onda
 * - 1: completo - display inteiro
 * Na completa é necessário limpar o display antes
 * Atualiza fisicamente o display antes de sair da função "display.display()"
 */
void update_display(uint8_t select)
{
  uint8_t y_pos;      // variável para salvar a posição no eixo y (vertical)

  if (select == 1)    // se for uma atualização completa do display
  {
    /*
     * seta para primeira coluna no eixo x, oitava linha no eixo y
     * texto tamanho 1 com fundo preto (apagando o que estiver por baixo)
     * carrega a palavra "Wave:"
     */
    display.setCursor(0,8);
    display.setTextSize(1);
    display.setTextColor(WHITE, BLACK);
    // salva a string na flash e realiza a leitura nela - economiza RAM
    display.println(F("Wave: "));
  }

  /*
   * Ajusta o tamanho do texto para 2
   * seta para a coluna 35 do eixo x
   * Escreve qual a forma de onda
   */
  display.setTextSize(2);
  display.setCursor(35,0);
  // lê a string no array armazenado na flash - economiza RAM
  display.println((__FlashStringHelper*)pgm_read_word(wave_text_1 + waveform));
  
  display.setTextSize(1);
  y_pos = 32;

  /*
   * Atualiza as linhas para os seguintes itens
   * - Frequencia, Amplitude, Offset, número de amostras
   * Lê as strings salvas na flash - economizando RAM
   */
  uint8_t count;
  for (count = 0; count < 4; count++)
  {
    /*
     * Se não for DC, atualiza todas as 4 linhas
     * - caso for "dc", apaga os valores de Freq e Amp
     * - pula para linha de Voff
     */
    if (waveform != DC)
    {
      display.setCursor(0,y_pos);
      if (select == 1)
      {
        display.println( (__FlashStringHelper*)pgm_read_word(parameter_text + count*2 + 0) );
      }
      display.setCursor(36,y_pos);
      display.print(wave_parameters[count]);
      display.println( (__FlashStringHelper*)pgm_read_word(parameter_text + count*2 + 1) );
      y_pos += 8;
    }
    /*
     * Caso for "dc"
     */
    else  
    {
      if (count < OFFSET)
      {
        if (select == 1)
        {
          display.print( (__FlashStringHelper*)pgm_read_word(parameter_text + count*2 + 0) );
        }
        display.setCursor(36,y_pos);
        display.println(F("        "));
        y_pos += 8;
      }
      else
      {
        if (select == 1)
        {
          display.print( (__FlashStringHelper*)pgm_read_word(parameter_text + count*2 + 0) );
        }
        display.setCursor(36,y_pos);
        display.print(wave_parameters[count]);
        display.println( (__FlashStringHelper*)pgm_read_word(parameter_text + count*2 + 1) );
        y_pos += 8;
      }
    }
  }
   
  display.display();
}

/*
 * Tela de boas vindas
 */
void splash_display()
{
  display.setCursor(0,8);
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);     // apaga o fundo
  display.println(F("mini "));

  display.setTextSize(2);
  display.setCursor(27,0);
  display.println(F("Wave_Gen"));

  display.setTextSize(1);
  display.println();
  display.println(F("by: Haroldo Amaral"));
  display.println(F("agaelema@gmail.com"));

  display.println();
  display.println(F("blog.filipeflop.com"));

  display.display();
  delay(2000);
}

/*
 * Funções chamadas dentro das interrupções das chaves
 * - apenas incrementam os flags de estado
 */
void SW1_interrupt()
{
  SW1_state++;
}
void SW2_interrupt()
{
  SW2_state++;
}

