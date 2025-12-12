//Sara Pavey and Luka Wozniak
//CPE Swamp Cooler Final Project
//12-12-2025

#define RDA 0x80
#define TBE 0x20  

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

void adc_init();
unsigned int adc_read(unsigned char);
void U0putchar(unsigned char);
void U0Init(int);

void setup() {
  unsigned char* ddr_b = (unsigned char*) 0x24; //Memory Mapped I/O
  unsigned char* port_b = (unsigned char*) 0x25; 
  unsigned char* ddr_h = (unsigned char*) 0x101;
  unsigned char* port_h = (unsigned char*) 0x102;
  unsigned char* ddr_e = (unsigned char*) 0x2D;
  unsigned char* port_e = (unsigned char*) 0x2E;
  unsigned char* ddr_g = (unsigned char*) 0x33;
  unsigned char* port_g = (unsigned char*) 0x34;
  unsigned char* ddr_a = (unsigned char*) 0x21;
  unsigned char* port_a = (unsigned char*) 0x22;
  unsigned char* ddr_f = (unsigned char*) 0x30;
  unsigned char* port_f = (unsigned char*) 0x31;
  *ddr_b |= (1 << PB7); //configure pin 13 as output, IN4
  *ddr_b |= (1 << PB6); //configure pin 12 as output, IN3
  *ddr_b |= (1 << PB5); //configure pin 11 as output, IN2
  *ddr_b |= (1 << PB4); //configure pin 10 as output, IN1
  *ddr_h |= (1 << PH6); //configure pin 9 as output, breadboard
  *ddr_h |= (1 << PH5); //configure pin 8 as output, LCD RS
  *ddr_h |= (1 << PH4); //configure pin 7 as output, LCD E
  *ddr_h |= (1 << PH3); //configure pin 6 as output, LCD D4
  *ddr_e |= (1 << PE3); //configure pin 5 as output, LCD D5
  *ddr_g |= (1 << PG5); //configure pin 4 as output, LCD D6
  *ddr_e |= (1 << PE5); //configure pin 3 as output, LCD D7
  *ddr_e &= ~(1 << PE4); //configure pin 2 as input, water sensor data/S
  *ddr_a |= (1 << PA0); // configure pin 22 as output, blue LED
  *ddr_a |= (1 << PA2); // configure pin 24 as output, red LED
  *ddr_a |= (1 << PA4); // configure pin 26 as output, yellow LED
  *ddr_a |= (1 << PA6); // configure pin 28 as output, green LED
  *ddr_f &= ~(1 << PF1); // configure pin A1 as input, temp sensor
}

void loop() {
  // put your main code here, to run repeatedly:

}

void adc_init(){

  *my_ADCSRA |= 0b10000000; //Enable ADC
  *my_ADCSRA &= 0b11011111; //Disable auto-trigger
  *my_ADCSRA &= 0b11110111; //Disable ADC interrupt
  *my_ADCSRA &= 0b11111000; //Set prescaler to 2

  *my_ADCSRB &= 0b11110111;
  *my_ADCSRB &= 0b11111000;
  
  *my_ADMUX &= 0b01111111;
  *my_ADMUX |= 0b01000000;
  *my_ADMUX &= 0b11011111;
  *my_ADMUX &= 0b11100000;
}

unsigned int adc_read(unsigned char adc_channel_num) {

  *my_ADMUX &= 0b11100000; // clear the channel selection bits (MUX 4:0)

  *my_ADCSRB &= 0b11011111; // clear the channel selection bits (MUX 5)

  *my_ADMUX |= (adc_channel_num & 0x07); //for setting new channel
  
  *my_ADCSRA |= 0b01000000; // set bit 6 of ADCSRA to 1 to start a conversion

  while((*my_ADCSRA & 0x40) != 0); // wait for the conversion to complete
  
  my_ADC_DATA = (unsigned short *) 0x78; // return the result in the ADC data register and format the data based on right justification
  unsigned int val = (*my_ADC_DATA & 0x03FF);
  return val;
}

void U0putchar(unsigned char U0pdata){

  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void U0Init(int U0baud){

  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

