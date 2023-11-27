//Classic PID controller created by Alejandro Dumas

//This code works with duyal driver DBH-12V and similars 

double target_position = 0;
double motor_position = 0;

//------------PID GAINS----------------- 
double kp = 0.4;   //1.5   //0.2
double ki = 0.0001;
double kd = 0.7;  //25.5;   //2.500;     //2

//----------Controller variables-------

int dt;
unsigned long t;
unsigned long t_prev = 0;

double e, e_prev = 0, integral, integral_prev = 0;
double Vmax = 500;
double Vmin = -500;
double V = 0;

const byte DirPin1 = 10;  //PWM pin IN1A
const byte DirPin2 = 11;  //PWM pin IN2A

double sample_counter = 0;          //Sample counter → To create a database we identify the number of sample, not necesary to controller



//-----------Motor Driver Function---------------

void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 50) {
    PWMval = 50;                   //Make sure dont pass the pwm max value that arduino supports
  }
  if (V > 0) {
    analogWrite(DirPin1, PWMval);
    digitalWrite(DirPin2, LOW);
  }
  else if (V < 0) {
    
    digitalWrite(DirPin1, LOW);
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
   t = millis();                                        //Start the sample time
   dt = (t - t_prev);                                   //Step time → the time between each sample

   e = target_position - motor_position;                                       // Error
   integral = integral_prev + ((dt * (e + e_prev)) / 2);        // Integration of Error → for intehral controller
  
   V = (kp * e) + (ki * integral) + (kd * (e - e_prev) / dt) ;    // Controlling Function

    if (V > Vmax) {
      V = Vmax;
    }

    if (V < Vmin) {
      V = Vmin;
    }

    //------- Save past values---------------- 
    integral_prev = integral;       //Save the last value of integral function      
    e_prev = e;
    t_prev = t;
    WriteDriverVoltage(V, Vmax);

    //Serial.print(sample_counter); Serial.print("  ");
    Serial.print(motor_position); Serial.print("  ");
    Serial.print(target_position); Serial.print("  ");
    //Serial.print(dt); Serial.print("  ");
    Serial.print(V); Serial.print("  \n");
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
