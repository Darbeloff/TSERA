#include <SPI.h>

// Slave Select pins for encoders 1, 2 and 3
// Feel free to reallocate these pins to best suit your circuit
const int slaveSelectEnc1 = 53;
const int slaveSelectEnc2 = 47;
const int slaveSelectEnc3 = 49;
const int slaveSelectEnc4 = 51;



void initEncoder(int encoderName){
  // Set slave selects as outputs
  SPI.begin();
  
  pinMode(encoderName, OUTPUT);

  // Raise select pins
  // Communication begins when you drop the individual select signals
  digitalWrite(encoderName,HIGH);

  // Initialize encoder
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(encoderName,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(encoderName,HIGH);       // Terminate SPI conversation 
  
  }


void initEncoders() {
  SPI.begin();
  initEncoder(slaveSelectEnc1);
  initEncoder(slaveSelectEnc2);
  initEncoder(slaveSelectEnc3);
  initEncoder(slaveSelectEnc4);
}


void clearEncoderCount(int encoderName){
  // Set encoder's data register to 0
  digitalWrite(encoderName,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(encoderName,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder's current data register to center
  digitalWrite(encoderName,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(encoderName,HIGH);     // Terminate SPI conversation
  }


void clearEncoderCounts() {
  clearEncoderCount(slaveSelectEnc1);
  clearEncoderCount(slaveSelectEnc2);
  clearEncoderCount(slaveSelectEnc3);
  clearEncoderCount(slaveSelectEnc4);
}


long readEncoder(int encoderName) {

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value; 
   
  noInterrupts();           // disable all interrupts
  
  // Read encoder
  digitalWrite(encoderName,LOW);      // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  SPI.endTransaction();
  digitalWrite(encoderName,HIGH);     // Terminate SPI conversation 
  
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;

  interrupts();             // enable all interrupts
  
  return count_value;
}


