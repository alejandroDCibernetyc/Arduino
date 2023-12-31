//Classic PID controller created by Alejandro Dumas → alejandroDCibernetyc

//This code works with duyal driver DBH-12V and similars 

double target_position = 0;
double motor_position = 0;
//------------Sampling Times----------------- 
double Ts = 0.00066;       //Sampling period
double Ti = 1;     //proposed time
double Td = 1;	  //proposed time


//------------PID GAINS----------------- 

double k = 500000;      //Declaring the main gain, to change the gains just change ths value, others K gains depends in this value
double kp = k;   
double ki = 0;//k/Ti;
double kd = 0.00001;//k*Td;  
//------------Constant definition-----------------

double a = kp + ((ki*Ts)/2) + (kd/Ts);
double b = -kp + ((ki*Ts)/2)  - ((2*kd)/Ts);
double c = (kd/Ts);


//----------Controller variables-------

//----------Definition Controller variables-------

double u, u_prev = 0;       //Controller signal
double e = 0;               //Current error
double e_prev = 0;          //Previous error
double e_2prev = 0;         //error prior to previous error

double Vmax = 500;
double Vmin = -500;

double sample_counter = 0;          //Sample counter → To create a database we identify the number of sample, not necesary to controller
//double V = 0;

const byte DirPin1 = 10;  //PWM pin IN1A
const byte DirPin2 = 11;  //PWM pin IN2A



//-----------Motor Driver Function---------------

void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 20) {
    PWMval = 20;                   //Make sure dont pass the pwm max value that arduino supports
  }
  if (V > 0) {
  
    analogWrite(DirPin1, PWMval);
    analogWrite(DirPin2, 0);     

  }
  else if (V < 0) {
    analogWrite(DirPin1, 0);
    analogWrite(DirPin2, PWMval);    
  }
  else {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
  }

}



volatile double encoders_position[] = {0,0};      //Array that contains the values of encoders, amount of ticks
const int encoder_pin_b[] = {4,5};                //Indicates the arduino pins that connects with encoders terminals B 
                                                  //Pin 4 for the encoder target, pin 5 for the motor encoder

void setup() {

  Serial.begin (9600);
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);  
  pinMode(encoder_pin_b[0], INPUT_PULLUP);    //pin B to encoder 1 (target encoder) WITH INTERNAL PULLUP
  pinMode(encoder_pin_b[1], INPUT_PULLUP);    //Pin B para encoder 2 (motor encoder)
  pinMode(2, INPUT_PULLUP);                   // internal pullup input pin   A encoder 1
  pinMode(3, INPUT_PULLUP);                   // internalL pullup input pin  A encoder 2
  
  
  //Setting up interrupt
  //A 1 rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin  2 on moust Arduino.
  attachInterrupt(0,readEncoder<0>,RISING);
  //B 2 rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin 3 on moust Arduino.
  attachInterrupt(1,readEncoder<1>,RISING);
  }
   
  void loop() {
 

   target_position = encoders_position[0];              //Desired Angular Position of the Motor
   motor_position = encoders_position[1];               //Actual Angular Position of the Motor
   
   e = target_position - motor_position;                                       // Error

   u = u + (a * e) + (b * e_prev) + (c + e_2prev);    // Controlling Function
  
    if (u > Vmax) {
      u = Vmax;
    }

    if (u < Vmin) {
      u = Vmin;
    }
    WriteDriverVoltage(u, Vmax);

    //------- Save past values---------------- 
    e_2prev = e_prev;       	//we must save the error before the previous error, first cycle this have to ve 0, in the second it have to ve 0 to, but in the third cyle it have to have the first error
    e_prev = e;			//saving previous error
    		//Saving previous function controller
    

    //Serial.print(sample_counter); Serial.print("  ");
    Serial.print(motor_position); Serial.print("  ");
    Serial.print(target_position); Serial.print("  ");
    Serial.print(e); Serial.print("  ");
    //Serial.print(dt); Serial.print("  ");
    Serial.print(u); Serial.print("  \n");
    sample_counter += 1;
    delay(10);

  
  }


template <int j>
void readEncoder(){
  int b = digitalRead(encoder_pin_b[j]);
  if(b > 0){
    encoders_position[j]++;
  }
  else{
    encoders_position[j]--;
  }
}
