//Sarah Pavey and Luka Wozniak
//CPE Swamp Cooler Final Project
//12-12-2025
void setup() {
  // put your setup code here, to run once:
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
  *ddr_b |= (1 << PB7); //configure pin 13 as output
  *ddr_b |= (1 << PB6); //configure pin 12 as output
  *ddr_b |= (1 << PB5); //configure pin 11 as output
  *ddr_b |= (1 << PB4); //configure pin 10 as output
  *ddr_h |= (1 << PH6); //configure pin 9 as output
  *ddr_h |= (1 << PH5); //configure pin 8 as output
  *ddr_h |= (1 << PH4); //configure pin 7 as output
  *ddr_h |= (1 << PH3); //configure pin 6 as output
  *ddr_e |= (1 << PE3); //configure pin 5 as output
  *ddr_g |= (1 << PG5); //configure pin 4 as output
  *ddr_e |= (1 << PE5); //configure pin 3 as output
  *ddr_e &= ~(1 << PE4); //configure pin 2 as input
  *ddr_a |= (1 << PA0); // configure pin 22 as output
  *ddr_a |= (1 << PA2); // configure pin 24 as output
  *ddr_a |= (1 << PA4); // configure pin 26 as output
  *ddr_a |= (1 << PA6); // configure pin 28 as output
  *ddr_f &= ~(1 << PF1); // configure pin A1 as input
}

void loop() {
  // put your main code here, to run repeatedly:

}
