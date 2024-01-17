//This code can read the steps of rotary encoder in both sence, using PCI
//The code just allows to use one encoder, but it can be more
const byte wireB = 8;     //Indicates the pin number to connect wire B of the encoder
const byte wireA = 9;     //Connect wire A
// Double to store the encoder steps
volatile double encoder_value = 0;      
volatile bool valueA, valueB; 
volatile bool prev_valueA, prev_valueB; 
 
void setup() {
  Serial.begin(9600);
  // Set wireB as input
  pinMode(wireB, INPUT_PULLUP);
  pinMode(wireA, INPUT_PULLUP);
  prev_valueA = digitalRead(wireA);  //Set initial value of wire A and B
  prev_valueB = digitalRead(wireB);

 
  // Enable PCIE2 Bit1 = 1 (Port B)
  PCICR |= B00000001;
  // Select (PCINT0)Pin Change Mask 0 - PSMK0 Bit0 = 1 and Bit1 = 1 (Pin B0, B1)→ pin8, pin9 arduino uno board
  PCMSK0 |= B00000011;
 
}

void loop() {
 
   Serial.println(encoder_value);
}
 
ISR (PCINT0_vect)
{
  // Interrupt for Port B
  valueA = digitalRead(wireA);
  valueB = digitalRead(wireB);

  if ( (prev_valueA == HIGH && prev_valueB == HIGH && valueA == HIGH && valueB == LOW )||
        (prev_valueA == HIGH && prev_valueB == LOW && valueA == LOW && valueB == LOW )||
        (prev_valueA == LOW && prev_valueB == LOW && valueA == LOW && valueB == HIGH )||
        (prev_valueA == LOW && prev_valueB == HIGH && valueA == HIGH && valueB == HIGH )){
          //If any condition is true increase the encoder value
          encoder_value++;
          prev_valueA = valueA;
          prev_valueB = valueB;

  }
  else if( (prev_valueA == LOW && prev_valueB == HIGH && valueA == LOW && valueB == LOW )||
        (prev_valueA == LOW && prev_valueB == LOW && valueA == HIGH && valueB == LOW )||
        (prev_valueA == HIGH && prev_valueB == LOW && valueA == HIGH && valueB == HIGH )||
        (prev_valueA == HIGH && prev_valueB == HIGH && valueA == LOW && valueB == HIGH )){
          encoder_value--;
          prev_valueA = valueA;
          prev_valueB = valueB;
  }
  else{
    //Do nothing
  }
 
}
 