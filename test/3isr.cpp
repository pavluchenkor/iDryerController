// By John Main © best-microcontroller-projects.com
// Using Pin Change interrupts.
#include <Arduino.h>

const byte DT = A1;
const byte CLK = A2;
const byte encBut = A3;

volatile static byte oldPorta;
volatile static byte PCMask;
volatile byte flagISR = 0, intsFound = 0;

byte isr_1 = 0;
byte isr_2 = 0;
byte isr_3 = 0;


void setup() {

  Serial.begin(57600);

  pinMode(DT, INPUT_PULLUP);
  pinMode(CLK, INPUT_PULLUP);
  pinMode(encBut, INPUT_PULLUP);

  oldPorta = PINC;
  PCMSK1 |= (1<<PCINT9);
  PCMSK1 |= (1<<PCINT10);
  PCMSK1 |= (1<<PCINT11);
  Serial.println(PCMSK1,HEX); // What is the total mask.
  PCMask = PCMSK1;

  PCICR |= (1<<PCIE1); // Enable PCINT on portc.
}

//////////////////////////////////////////////
// Arduino Delay LED flash.
void loop(){

  // delay(delayTime);
  //digitalWrite(LED,HIGH);
  // delay(delayTime);
  //digitalWrite(LED,LOW);

  if(flagISR) {
    // Serial.print("Found ");Serial.print(intsFound);
    // Serial.print(" interrupts, using: ");Serial.println(flagISR);

    // if(intsFound>1)
    //   Serial.println("-----------> 1");
    //    //tone(TONE_PIN,600,100);
    // else
    //   Serial.println("-----------> 2");
    //    //tone(TONE_PIN,300,100);
    if(isr_1) Serial.println("-----------> isr_1");
    if(isr_2) Serial.println("-----------> isr_2");
    if(isr_3) Serial.println("-----------> isr_3");
    
    // intsFound = flagISR = 0;
    flagISR = 0;
    isr_1 = isr_2 = isr_3 = 0;
  }
}

ISR( PCINT1_vect ) {
  byte change, v1, v2, v3;
  change = oldPorta ^ PINC;

  v1 = oldPorta & (1<<PCINT9);
  v2 = oldPorta & (1<<PCINT10);
  v3 = oldPorta & (1<<PCINT11);

  // v1==0 previously so detecting rising edge.
  if (v1==0 && change & (1<<PCINT9)) isr_1 = 1;
  if (v2==0 && change & (1<<PCINT10)) isr_2 = 1;
  if (v3==0 && change & (1<<PCINT11)) isr_3 = 1;
  if (change && (v1 == 0 || v2 == 0 || v3 == 0) ) {
    // intsFound++; // Only count rising edge interrupts (as this interrupt reacts to both).
    flagISR = 1; // rising edge only
  }
  oldPorta = PINC;
}
