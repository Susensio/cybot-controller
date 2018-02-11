
// Written by Nick Gammon
// February 2011


#include <SPI.h>
const byte SPI_INTERRUPT_PIN = 3;

char buf [10];
volatile byte pos;
volatile byte parityByte;
volatile boolean spiInterrupt;

int16_t linearVelocityRef = 0;
int16_t angularVelocityRef = 0;

void setup (void)
{
  Serial.begin (115200);   // debugging
  Serial.println("Hola");

  // turn on SPI in slave mode
  SPCR |= bit (SPE);

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  
  // get ready for an interrupt 
  pos = 0;   // buffer empty
  spiInterrupt = false;

  // now turn on interrupts
  SPI.attachInterrupt();
  attachInterrupt(digitalPinToInterrupt(SPI_INTERRUPT_PIN), spiDataReady, RISING);

}  // end of setup

void spiDataReady() {
    spiInterrupt = true;
}

// SPI interrupt routine
ISR (SPI_STC_vect) {
  buf [pos++] = SPDR; // grab byte from SPI Data Register
}

// main loop - wait for flag set in interrupt routine
void loop (void)
{
  if (spiInterrupt) { 
    // Parity check
    byte parityByte = buf[pos-1];
    byte parity = 0;
    for (int i=0; i<pos-1; i++){
      parity += buf[i];
    }

    // Data received correctly
    if (parityByte == parity){
      Serial.println ('!');
      Serial.println (buf[0]);
      linearVelocityRef = 0;
      if (buf[0] == 'M'){
        //linearVelocityRef = buf[1];
        //linearVelocityRef <<= 8;
        //linearVelocityRef |= buf[2] & 0xFF;
        linearVelocityRef = (buf[1]<<8) | buf[2] & 0xFF;
        angularVelocityRef = (buf[3]<<8) | buf[4];
        
        Serial.println (linearVelocityRef); 
        Serial.println (angularVelocityRef);  
      }
    }

    // Reset buffer
    pos = 0;
    spiInterrupt = false;
    }
    
}  // end of loop
