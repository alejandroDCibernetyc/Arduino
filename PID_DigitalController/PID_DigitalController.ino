//Digitakl PID controller created by Alejandro Dumas

#include "Wire.h"
#define PCF8591 (0x90 >> 1) // I2C bus address

//PIN A4 → SDA
//PIN A5 → SCL

double target_voltage = 0;
double current_voltage = 0;

//------------Sampling Times----------------- 
double Ts = 5;       //Sampling period
double Ti = 0.5;     //proposed time
double Td = 0.1;	  //proposed time

//------------PID GAINS----------------- 

double k = 0.8;      //Declaring the main gain, to change the gains just change ths value, others K gains depends in this value
double kp = k;   
double ki = k/Ti;
double kd = k*Td;  

//------------Constant definition-----------------

double a = kp + (ki*Ts)/2 + kd/Ts;
double b = -kp + (ki+Ts)/2  + (2*kd)/Ts;
double c = kd/Ts;

//----------Definition Controller variables-------



double u, u_prev = 0;       //Controller signal
double e = 0;               //Current error
double e_prev = 0;          //Previous error
double e_2prev = 0;         //error prior to previous error

double Vmax = 1023;
double Vmin = 0;
double V = 0;

double sample_counter = 0;          //Sample counter → To create a database we identify the number of sample, not necesary to controller



//-----------PCF8591 DAC Function---------------

void analogOutputVoltage(double u, double Vmax) {
  int PWMval = int(255 * abs(u) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;                   //Make sure dont pass the pwm max value that arduino supports
  
  Wire.beginTransmission(PCF8591); 	// wake up PCF8591
  Wire.write(0x40); 			// control byte - turn on DAC (binary 1000000)
  Wire.write(PWMval); 			// value to send to DAC
  Wire.endTransmission(); 		// end tranmission
}


void setup() {

  Serial.begin (9600);		//Init serial port to show data
  Wire.begin();			//Init wire to comunicate to DAC

  
  }
   
  void loop() {
 

   target_voltage = analogRead(0);   	//Desired voltage, connect pin A0 to the signal input of RLC
   current_voltage = analogRead(1);     //Actual voltage, connect A1 to the capacitor voltage of RLC	 


   e = target_voltage - current_voltage;                   // Error

  
   u = u_prev + (a * e) + (b * e_prev) + (c + e_2prev);    // Controlling Function

    analogOutputVoltage(u, Vmax);


    //------- Save past values---------------- 
    e_2prev = e_prev;       	//we must save the error before the previous error, first cycle this have to ve 0, in the second it have to ve 0 to, but in the third cyle it have to have the first error
    e_prev = e;			//saving previous error
    u_prev = u;			//Saving previous function controller

    Serial.print(sample_counter); Serial.print("  ");
    Serial.print(current_voltage); Serial.print("  ");
    Serial.print(target_voltage); Serial.print("  ");
    //Serial.print(dt); Serial.print("  ");
    Serial.print(u); Serial.print("  \n");
    sample_counter += 1;
    delay(10);

  
  }

