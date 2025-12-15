//Sara Pavey and Luka Wozniak
//CPE Swamp Cooler Final Project
//12-12-2025

#include <LiquidCrystal.h> //lcd
#include <DHT.h> //temperature
#include <DHT_U.h> //temperature
#include <RTClib.h> //clock/time
#include <Keypad.h> //matrix keypad used for stop and reset buttons
#include <Stepper.h> //stepper
#include <Wire.h> //set

#define RDA 0x80
#define TBE 0x20  

#define WATER_ADC_CH 2

#define DHTPIN 46
#define DHTTYPE DHT11

#define WATER_THRESHOLD 2 //best value in testing
#define TEMP_THRESHOLD 24.0 //in Celsius, chosen to be somewhat above temp of my colder room

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

int stepsPerRev = 2000;
int rpm = 10;
Stepper stepperVent (stepsPerRev, 47, 51, 49, 53);

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

typedef enum state {DISABLED, IDLE_STATE, ERROR, RUNNING} state;
volatile state activeState = DISABLED; //System will not start off running the cooler 

unsigned long previousMillis = 0;  // will store last time LED was updated. Code snippet from ArduinoDocs
const unsigned long interval = 60000;  // interval at which to blink (milliseconds); 60000 ms

void adc_init();
int adc_read(char);
void U0putchar(unsigned char);
static void U0putint_recursive(unsigned int);
void U0putint(unsigned int);
void U0init(int);
void setLED(state);
void startbuttonISR();
int getWaterLevel();
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

  stepperVent.setSpeed(rpm);

  Wire.begin();
  if (!rtc.begin()) {
    lcd.print("Clock Error");
  }

  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  setLED(DISABLED);

}

void loop() {
  char k = keypad.getKey();
  static char lastk = 0;
  if (k == '1'){ //1 = Stop button: turn motor off (if on) and disable system
    updateState(DISABLED);
  }
  else if(activeState == ERROR && k == '2'){ //2 = Reset button
    if (waterThreshold()){
      updateState(IDLE_STATE);
    } 
  }

  if(activeState == DISABLED){
    lcd.clear(); //no reporting and no error message
    if(startPress){
      startPress = false;
      updateState(IDLE_STATE);
    }
  }
  else if (activeState == IDLE_STATE){
    if (!waterThreshold()){
      updateState(ERROR);
    }
    else if(tempThreshold()){ 
      updateState(RUNNING);
    }
    if (k != 0 && k != lastk){//If a new button press on keypad run step motor function
      stepMotor(k);
      lastk = k;
    }
    if (k == 0){
      lastk = 0;
    }
  }else if (activeState == ERROR){ 
    if (k != 0 && k != lastk){//If a new button press on keypad run step motor function
      stepMotor(k);
      lastk = k;
    }
    if (k == 0){
      lastk = 0;
    }
  }else{
    if (!waterThreshold()){
      updateState(ERROR);
    }else if(!tempThreshold()){ 
      updateState(IDLE_STATE);
    }
    if (k != 0 && k != lastk){//If a new button press on keypad run step motor function
      stepMotor(k);
      lastk = k;
    }
    if (k == 0){
      lastk = 0;
    }  
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
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

int adc_read(int channelnum) {

  ADMUX = (ADMUX & 0xF8) | (channelnum & 0x07);

  ADCSRA |= (1 << ADSC);

  while (ADCSRA & (1 << ADSC));

  return (int)ADC;
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
    case IDLE_STATE:
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

void startbuttonISR() {
  if (activeState == DISABLED) {
    startPress = true;
  }
}


int getWaterLevel(){
    int val = adc_read(WATER_ADC_CH);
    return val;
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
  static state lastState = DISABLED;

  lcd.clear();

  lcd.setCursor(0,0);

  if(infoState == ERROR){
    lcd.print("Error:");
    lcd.setCursor(0,1);
    lcd.print("Water low");
  }
  else{
    lcd.print("Temp:");
    lcd.print(getTemperature());
    lcd.print("C ");

    lcd.setCursor(0,1);
    lcd.print("Hum:");
    lcd.print(getHumidity());
    lcd.print("% ");
  }
}


void fanOff(){

  analogWrite(9, 0);
}

void fanOn(){

  analogWrite(9, 50);
}

void updateState(state newState){

  if(activeState == newState){
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

  switch(newState){
    case DISABLED:
      U0putchar('D');
      break;
    case IDLE_STATE:
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
  setLED(newState);

  if(newState == RUNNING){
    fanOn();
    logMotor();
    U0putchar('O');
    U0putchar('n');
    U0putchar('\n');
  }
  else if (activeState == RUNNING && newState != RUNNING){ //Only turn the fan off if it's on since this affects motor logs
    fanOff();
    logMotor();
    U0putchar('o');
    U0putchar('f');
    U0putchar('f');
    U0putchar('\n');
  }

  activeState = newState;
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

  logTime();
  U0putchar('V');
  U0putchar('e');
  U0putchar('n');
  U0putchar('t');
  U0putchar(':');
  U0putchar(' ');

  if(k == '3'){
    U0putchar('L');
    stepperVent.step(-stepsPerRev/4);
  }else if(k == '4'){
    U0putchar('R');
    stepperVent.step(stepsPerRev/4);
  }

  U0putchar('\n');
}
