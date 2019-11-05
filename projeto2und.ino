  //proximos passos:
//  comunicação
//  interrupcao
//  socket (EXTRA)

#define F_CPU 16000000UL
//#include <avr/io.h>
#include <avr/interrupt.h>
 
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

void intUSART();
void txByte(unsigned char info);
unsigned char rxByte(void);
void USART_Flush(void);
void comunicacaoRemota(char dados);
unsigned char dados;

volatile bool ligado = false;

int main (){
  uint16_t ad_valor =0;
  
  //Os pinos usados para interrupçao externa sao: PD2 e PD3
  // entao e necessario configura-los como entrada, ja que vamos usar esses
  // pinos para controlar os botoes
  
  // DEFININDO AS ENTRADAS 6 E 7 (DOIS BOTÕES)
//  DDRD &= 0b00111111;
  
  //Configurando pinos 2 (PD2) como entradaa do botao de liga/desliga
  // e pino 6 como entrada do botao para leitura do potenciometro
  DDRD &=  0b10111011;
  
  // DEFININDO A SAIDA (CONTROLE DO MOTOR)
  DDRB |= 0b00011111;
  
  // CONFIGURANDO BOTÕES EM PULL-UP E SAIDA 0
  //PORTD |= 0b11000000;
  PORTD |= 0b01000100; // pull-up pinos 2 e 6
  PORTB &= 0b11111110;

  // CONFIGURANDO OS TIMERS
  TCCR1A |= 0b10000001;
  TCCR1B |= 0b00001001;
  OCR1A = 0;

  // QUAL VAI SER O VALOR DO OCR?
  // 
  
  //Configurando interrupcao externa
  EICRA &= 0b00001111; // o sinal logico baixo gera uma interrupcao

  // Habilitando interrupcao do pino 2 (INT0)
  EIMSK |= 0b00000001; 
  
  intUSART();
  
  //CONFIGURANDO ADC (ENTRADA POTECIOMENTO)
  
  ADMUX |= 0b01000000; //AVCC como referencia, ADLAR=0, MUX = 0000 (ADC0 como entrada)
  
  ADCSRA |= 0b10000111; //Enable do ADC, Escalador de Clock 120

  sei(); //habilita interrupcao global

  while(1){
    if(ligado && !(PIND & 0b01000000)){ // Ler potenciometro e joga na saida PWM
      ADCSRA |= 0b01000000;
      while(!(ADCSRA & 0b00010000));
      ad_valor = ADC;
      OCR1A =((uint8_t)ceil((255.0*ad_valor)/1023.0)); // Saida PWM pino 11
    }
    else{
        dados = rxByte();
        comunicacaoRemota(dados);
      }
  }
}

void comunicacaoRemota(unsigned char dados){
      if(ligado){
          if(dados == (unsigned char)'0'){
//            Serial.println("MOTOR DESLIGADO");/
            OCR1A = (uint8_t)(0);
            ligado = false;
          }
          if(dados == (unsigned char)'1'){
//            Serial.println("MOTOR LIGADO 50%");/
            OCR1A = (uint8_t)(128);
          }
          else if(dados == (unsigned char)'2'){
//            Serial.println("MOTOR LIGADO 25%"); /
            OCR1A = (uint8_t)(64);
          }
          else if(dados == (unsigned char)'3'){
//            Serial.println("MOTOR LIGADO 75%"); /
            OCR1A = (uint8_t)(191);
          }
          else if(dados == (unsigned char)'4'){
//            Serial.println("MOTOR LIGADO 100%/"); 
            OCR1A = (uint8_t)(255);
          }
        }
       else{
          if(dados == (unsigned char)'1'){
//          Serial.println("MOTOR LIGADO"); /
          OCR1A = (uint8_t)(128);
          ligado = true;
         }
        }
  }
  
void intUSART()
{
  // Configurado para 38.4kbps
//  UBRR0L = 25;
//  UBRR0H = 0;

  UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
  UBRR0L = (uint8_t)(BAUD_PRESCALLER);
  // U2X=1 - Dobra a velocidade
  //UCSRA = (1<<U2X);

  // UCSZ2 = 0 - 8 bits
  UCSR0B |= _BV(RXEN0)|_BV(TXEN0);

  // UCSZ1 = 1 e UCSZ0 = 1 -> 8 bits
  // USBS0 = 0 -> 1 - stop bits
  // UPM0 = 0 e UMP0 = 0 -> sem bit de paridade
  UCSR0C |= _BV(UCSZ01)|_BV(UCSZ00);
}

void txByte(unsigned char info)
{
  // Transmissão de dados
  //Bit UDRE indica quando o buffer de tx pode enviar dados
  while(!(UCSR0A & (1<<UDRE0)));
  UDR0 = info;
}

unsigned char rxByte(void)
{
  //Bit RXC sinaliza quando existem bytes não lidos no buffer
  while(!(UCSR0A & (1<<RXC0)));
  return UDR0;
}
void USART_Flush( void )
{
 unsigned char dummy;
 while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
}

ISR(INT0_vect){//Liga o sistema
    ligado = !ligado;
    if(ligado){
       OCR1A = (uint8_t)(128);
       txByte((unsigned char)'1');
       txByte((unsigned char)'\n');       
    }
    else{
       OCR1A = (uint8_t)(0);
       txByte((unsigned char)'0');
       txByte((unsigned char)'\n');
    }
}
