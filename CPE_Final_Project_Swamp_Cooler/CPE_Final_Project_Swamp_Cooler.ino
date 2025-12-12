//Sara Pavey and Luka Wozniak
//CPE Swamp Cooler Final Project
//12-12-2025
#include <LiquidCrystal.h> //lcd
#include <DHT.h> //temperature
#include <DHT_U.h> //temperature
#include <RTClib.h> //clock/time
#include <Keypad.h> //matrix keypad used for stop and reset buttons

#define RDA 0x80
#define TBE 0x20  

#define TEMP_ADC_CH 1
#define WATER_ADC_CH 2

#define DHTPIN 46
#define DHTTYPE DHT11

#define WATER_THRESHOLD 250 //chosen to mimic lab 8
#define TEMP_THRESHOLD 25.0 //in Celsius, chosen to be somewhat above temp of my colder room

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0 = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

volatile unsigned char* ddr_b = (unsigned char*) 0x24; 
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_h = (unsigned char*) 0x101; 
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_e = (unsigned char*) 0x2D;
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_g = (unsigned char*) 0x33;
volatile unsigned char* port_g = (unsigned char*) 0x34;
volatile unsigned char* ddr_a = (unsigned char*) 0x21;
volatile unsigned char* port_a = (unsigned char*) 0x22;
volatile unsigned char* ddr_f = (unsigned char*) 0x30;
volatile unsigned char* port_f = (unsigned char*) 0x31;

LiquidCrystal lcd(8, 7, 6, 5, 4, 3); //Setup LCD in 4 pin mode
DHT dht(DHTPIN, DHTTYPE); //temperature
int motorstate = 0; //Set the stepper motor state to zero
RTC_DS1307 rtc; //clock

volatile bool startPress = false; //start button
const byte ROWS = 4; 
const byte COLS = 4;
char startKeypad[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {30, 32, 34, 36};
byte colPins[COLS] = {38, 40, 42, 44};
Keypad keypad = Keypad(makeKeymap(startKeypad), rowPins, colPins, ROWS, COLS);

typedef enum {DISABLED, IDLE, ERROR, RUNNING} state;
volatile state activeState = DISABLED; //System will not start off running the cooler 

unsigned long previousMillis = 0;  // will store last time LED was updated. Code snippet from ArduinoDocs
const unsigned long interval = 60000;  // interval at which to blink (milliseconds); 60000 ms = 1min

void adc_init();
unsigned int adc_read(unsigned char);
void U0putchar(unsigned char);
static void U0putint_recursive(unsigned int);
void U0putint(unsigned int);
void U0init(int);
void setLED(state);
void startbuttonISR();
unsigned int getWaterLevel();
bool waterThreshold();
float getTemperature();
bool tempThreshold();
float getHumidity();
void displayLCDInfo(state);
void fanOff();
void fanOn();
void updateState(state);
void logMotor();
void logTime();
void stepMotor();

void setup() {
  *ddr_b |= (1 << PB7); //configure pin 13 as output, IN4
  *ddr_b |= (1 << PB6); //configure pin 12 as output, IN3
  *ddr_b |= (1 << PB5); //configure pin 11 as output, IN2
  *ddr_b |= (1 << PB4); //configure pin 10 as output, IN1
  *ddr_h |= (1 << PH6); //configure pin 9 as output, motor
  *ddr_h |= (1 << PH5); //configure pin 8 as output, LCD RS
  *ddr_h |= (1 << PH4); //configure pin 7 as output, LCD E
  *ddr_h |= (1 << PH3); //configure pin 6 as output, LCD D4
  *ddr_e |= (1 << PE3); //configure pin 5 as output, LCD D5
  *ddr_g |= (1 << PG5); //configure pin 4 as output, LCD D6
  *ddr_e |= (1 << PE5); //configure pin 3 as output, LCD D7
  *ddr_a |= (1 << PA0); // configure pin 22 as output, blue LED
  *ddr_a |= (1 << PA2); // configure pin 24 as output, red LED
  *ddr_a |= (1 << PA4); // configure pin 26 as output, yellow LED
  *ddr_a |= (1 << PA6); // configure pin 28 as output, green LED
  *ddr_f &= ~(1 << PF1); // configure pin A1 as input, temp sensor
  *ddr_f &= ~(1 << PF2); //configure pin A2 as input, water sensor data/S

  *port_b |= (1 << PB7); //Send a 1 to set stepper motor to 0 position
  *port_b &= ~(1 << PB6); //Send a 0 to set stepper motor to 0 position
  *port_b &= ~(1 << PB5); //Send a 0 to set stepper motor to 0 position
  *port_b &= ~(1 << PB4); //Send a 0 to set stepper motor to 0 position
  lcd.begin(16, 2); //clear and set position to (0,0)

  U0init(9600);

  adc_init();

  *ddr_e &= ~(1 << PE4); //configure pin 2 as input, Start ISR Button
  *port_e |= (1 << PE4); //pullup, implemented with code instead of resistor since board was crowded
  attachInterrupt(digitalPinToInterrupt(2), startbuttonISR, FALLING);

  dht.begin();

  if (!rtc.begin()) {
    lcd.print("Clock Error");
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //Comment out after first successful upload of sketch

}

void loop() {
  char k = keypad.getKey();
  static char lastk = 0;
  if (k == '1'){ //1 = Stop button: turn motor off (if on) and disable system
    updateState(DISABLED);
  }
  else if(activeState == ERROR && k == '2'){ //2 = Reset button
    if (waterThreshold()){
      updateState(IDLE);
    } 
  }
  switch(activeState){
    case DISABLED:
      lcd.clear(); //no reporting and no error message
      if(startPress){
        startPress = false;
        updateState(IDLE);
      }
      break;
    case IDLE: 
      if (!waterThreshold()){
        updateState(ERROR);
      }
      else if(tempThreshold()){ 
        updateState(RUNNING);
      }
      displayLCDInfo(activeState);
      if (k != 0 && k != lastk){//If a new button press on keypad run step motor function
        stepMotor(k);
        lastk = k;
      }
      if (k == 0){
        lastk = 0;
      }

      break;
    case ERROR:
      //Error message
      displayLCDInfo(activeState);
      if (k != 0 && k != lastk){//If a new button press on keypad run step motor function
        stepMotor(k);
        lastk = k;
      }
      if (k == 0){
        lastk = 0;
      }
      break;
    case RUNNING:
      if (!waterThreshold()){
        updateState(ERROR);
      }else if(!tempThreshold()){ 
        updateState(IDLE);
      }
      displayLCDInfo(activeState);
      if (k != 0 && k != lastk){//If a new button press on keypad run step motor function
        stepMotor(k);
        lastk = k;
      }
      if (k == 0){
        lastk = 0;
      }
      break;
  }
  //ArduinoDocs: Minute Loop
  if (activeState != DISABLED){
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval){
      previousMillis = currentMillis;
      displayLCDInfo(activeState);
    }
  }
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

static void U0putint_recursive(unsigned int n){ //lab 8 printing

    if (n == 0) {
        return;
    }
    U0putint_recursive(n / 10);
    U0putchar((n % 10) + '0');
}

void U0putint(unsigned int n1){

    if (n1 == 0) {
        U0putchar('0');
        return;
    }
    U0putint_recursive(n1);
}

void U0init(int U0baud){

  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

void setLED(state ledstate){

  *port_a &= ~((1 << PA0) | (1 << PA2) | (1 << PA4) | (1 << PA6)); //clearing LEDS out

  switch(ledstate){
    case DISABLED:
      *port_a |= (1 << PA4); //yellow
      break;
    case IDLE:
      *port_a |= (1 << PA6); //green
      break;
    case ERROR:
      *port_a |= (1 << PA2); //red
      break;
    case RUNNING:
      *port_a |= (1 << PA0); //blue
      break;
  }
}

void startbuttonISR(){

  startPress = true;
}

unsigned int getWaterLevel(){

    return adc_read(WATER_ADC_CH);
}

bool waterThreshold(){ //return true if water is at or above threshold

  return (getWaterLevel() >= WATER_THRESHOLD);
}

float getTemperature(){

  float temp = dht.readTemperature();
  return temp;
}

bool tempThreshold(){ //return true if temp is at or above threshold

  float temp1 = getTemperature();
  if (isnan(temp1)) return false; //failed sensor value
  return (temp1 >= TEMP_THRESHOLD);
}

float getHumidity(){

  float hum = dht.readHumidity();

  if (isnan(hum)) return -1.0; //failed sensor value

  return hum;
}

void displayLCDInfo(state infoState){
  lcd.clear();
  lcd.setCursor(0,0);
  if(infoState == ERROR){
      lcd.print("Error:");
      lcd.setCursor(0,1);
      lcd.print("Low Water");

      return;
  }
  lcd.print("Temp: ");
  lcd.print(getTemperature());
  lcd.print("°C");
  lcd.setCursor(0,1);
  lcd.print("Humid.: ");
  lcd.print(getHumidity());
  lcd.print("%");
}

void fanOff(){

  *port_h &= ~(1 << PH6); //Send a zero to pin 9 turning the motor off
}

void fanOn(){

  *port_h |= (1 << PH6); //Send a 1 to pin 9 turning the motor on
}

void updateState(state new){

  if(activeState == new){
    return;
  }

  logTime();

  U0putchar('S');
  U0putchar('t');
  U0putchar('a');
  U0putchar('t');
  U0putchar('e');
  U0putchar(':');
  U0putchar(' ');

  switch(new){
    case DISABLED:
      U0putchar('D');
      break;
    case IDLE:
      U0putchar('I');
      break;
    case ERROR:
      U0putchar('E');
      break;
    case RUNNING:
      U0putchar('R');
      break;
  }

  U0putchar('\n');
  setLED(new);

  if(new == RUNNING){
    fanOn();
    logMotor();
    U0putchar('O');
    U0putchar('n');
    U0putchar('\n');
  }
  else if (activeState == RUNNING && new != RUNNING){ //Only turn the fan off if it's on since this affects motor logs
    fanOff();
    logMotor();
    U0putchar('o');
    U0putchar('f');
    U0putchar('f');
    U0putchar('\n');
  }

  activeState = new;
}

void logMotor(){

  U0putchar('M');
  U0putchar('o');
  U0putchar('t');
  U0putchar('o');
  U0putchar('r');
  U0putchar(':');
  U0putchar(' ');
}

void logTime(){

  DateTime now = rtc.now();

  U0putint(now.year());
  U0putchar('-');
  U0putint(now.month());
  U0putchar('-');
  U0putint(now.day());
  U0putchar(' ');

  U0putint(now.hour());
  U0putchar(':');
  U0putint(now.minute());
  U0putchar(':');
  U0putint(now.second());
  U0putchar('\n');
}

void stepMotor(char k){

  if(activeState == DISABLED){
    return;
  }

  if(k == '3'){ //press 3 on matrix
    motorstate++;
  } 
  if(k == '4'){ //press 4 on matrix
    motorstate--;
  }

  if (motorstate > 3){
    motorstate = 0;
  }
  else if (motorstate < 0){
    motorstate = 3;
  }

  logTime();
  U0putchar('V');
  U0putchar('e');
  U0putchar('n');
  U0putchar('t');
  U0putchar(':');
  U0putchar(' ');

  if(k == '3'){
    U0putchar('L'); //Check if pressing 3 actually moves it left when testing circuit.
  }else if(k == '4'){
    U0putchar('R');
  }
  U0putchar(' ');
  U0putchar('P');
  U0putchar('o');
  U0putchar('s');
  U0putchar(':');
  U0putchar(' ');

  switch(motorstate){
    case 0:
      *port_b = (*port_b & 0x0F) | 0b10000000;
      U0putchar('0');
      break;
    case 1:
      *port_b = (*port_b & 0x0F) | 0b01000000;
      U0putchar('1');
      break;
    case 2:
      *port_b = (*port_b & 0x0F) | 0b00100000;
      U0putchar('2');
      break;
    case 3:
      *port_b = (*port_b & 0x0F) | 0b00010000;
      U0putchar('3');
      break;
    }

    U0putchar('\n');
}
