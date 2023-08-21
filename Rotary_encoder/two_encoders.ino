//This code allows you to connect two rotary enoder in one arduino 
//Compatible with Arduino Uno, Nano


//Contains the values of each encoder
volatile double encoderValue[] = {0, 0}; //Initialize values as 0

//Encoder Terminals
const int encoder1[] = {2, 4};   // encoder 1, pin2 → wire A, pin4→ Wire B
const int encoder2[] = {3, 5};   // encoder 2, pin3 → wire A, pin5→ Wire B

const int wireB[] = {encoder1[1], encoder2[1]};

void setup(){
    Serial.begin(9600);
    // Encoder 1
    pinMode(encoder1[0], INPUT_PULLUP);  
    pinMode(encoder1[1], INPUT_PULLUP);

    //Encder 2
    pinMode(encoder2[0], INPUT_PULLUP);
    pinMode(encoder2[1], INPUT_PULLUP);

    //Setting up external interrupts

    //Interruption 0 is attached o pin 2
    attachInterrupt(0,readEncoder<encoder1[1]>, RISING); //RISING pulse to the pin 2 (Int0) activate readEncoder

    //Interruption 1 is attached o pin 3
    attachInterrupt(1,readEncoder<encoder2[1]>, RISING); //RISING pulse to the pin 3 (Int1) activate readEncoder

}


void loop(){
    //Visualize encoders values
    Serial.print(encoderValue[0]);      //Print encoder 1 Value
    Serial.print(" ");
    Serial.println(encoderValue[1]);    //Print encoder 2 Value
}

template <int i>
void readEncoder(){                 //Evaluate de rotation direction
    int b = digitalRead(wireB[i]);
    if(b > 0){                      
        encoderValue[i]++;          //Increase the value for a sense
    }
    else{
        encoderValue[i]--;          //Decrease the value for the other sense
    }

}

