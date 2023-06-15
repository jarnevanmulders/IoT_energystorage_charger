#include <Arduino.h>
#include <SPI.h>



void setup() {

  pinMode(10, OUTPUT); // set the SS pin as an output
  SPI.begin();         // initialize the SPI library

}

void loop() {

  digitalWrite(10, LOW);            // set the SS pin to LOW

  SPI.transfer(0x00);             // send a write command to the MCP4131 to write at registry address 0x00
  SPI.transfer(100);              // send a new wiper value

  // for(byte wiper_value = 0; wiper_value <= 128; wiper_value++) {

  //   SPI.transfer(0x00);             // send a write command to the MCP4131 to write at registry address 0x00
  //   SPI.transfer(wiper_value);      // send a new wiper value

  //   Serial.println(analogRead(A0)); // read the value from analog pin A0 and send it to serial
  //   delay(1000); 
  // }

  digitalWrite(10, HIGH);           // set the SS pin HIGH

  delay(5000);

    digitalWrite(10, LOW);            // set the SS pin to LOW

  SPI.transfer(0x00);             // send a write command to the MCP4131 to write at registry address 0x00
  SPI.transfer(0);              // send a new wiper value

  // for(byte wiper_value = 0; wiper_value <= 128; wiper_value++) {

  //   SPI.transfer(0x00);             // send a write command to the MCP4131 to write at registry address 0x00
  //   SPI.transfer(wiper_value);      // send a new wiper value

  //   Serial.println(analogRead(A0)); // read the value from analog pin A0 and send it to serial
  //   delay(1000); 
  // }

  digitalWrite(10, HIGH);           // set the SS pin HIGH

  delay(5000);


}


