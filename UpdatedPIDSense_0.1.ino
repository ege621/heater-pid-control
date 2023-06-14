#include <RBDdimmer.h>
#define zerocross  2 
#define outputPin 3
#define SensorPin 2
unsigned long int avgValue;
int buf[10],temp;
int i,j;
float set_temperature = 43;  //Default temperature setpoint. Leave it 0 and control it with rotary encoder
float temperature_read = 0.0;
float PID_error = 0.01;
float previous_error = 0.0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
float last_set_temperature = 0;
//PID constants
//////////////////////////////////////////////////////////
int kp = 15;   int ki = 15;   int kd = 5; // Look up on the Ziegler Nichols method for tuning these
//////////////////////////////////////////////////////////
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;
int PID_values_fixed = 0;

dimmerLamp dimmer(outputPin); //initialase port for dimmer for MEGA, Leonardo, UNO, Arduino M0, Arduino Zero

void setup() {
 
  dimmer.begin(NORMAL_MODE, ON); //dimmer initialisation: name.begin(MODE, STATE)
  pinMode(11, INPUT);
  Serial.begin(9600);
  pinMode(5,OUTPUT);
  //tone(5,1000);

  pinMode (9, OUTPUT);  // output pin is fixed (OC1A)
  
}

void loop() {
  // First we read the real value of temperature
  temperature_read = read_ntc_10k(A0,11200);         
  //Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read;  //+-3
  //Calculate the P value
  PID_p =  0.1*kp * PID_error; //0.1
  //Calculate the I value in a range on +-3
  PID_i =  0.1*PID_i + (ki * PID_error);
  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000;
  //Now we can calculate the D calue
  PID_d = 0.01 * kd * ((PID_error - previous_error) / elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;
  PID_value = map(PID_value, 0, 255, 0, 100);
  //We define PWM range between 0 and 255
  PID_value = constrain(PID_value, 0, 90); //the controller bugs if set to %100
  //Now we can write the PWM signal to the mosfet on digital pin D3
  //Since we activate the MOSFET with a 0 to the base of the BJT, we write 255-PID value (inverted)
  //  analogWrite(PWM_pin,255-PID_value);
  dimmer.setPower(PID_value); // name.setPower(0%-100%)
  previous_error = PID_error;     //Remember to store the previous error for next loop.
 
            
  Serial.print(temperature_read);
  Serial.print(" ");
  Serial.print(set_temperature);
  Serial.print(" ");
  Serial.print(PID_value);
  Serial.print(" ");
  Serial.print(kp);
  Serial.print(" ");
  Serial.print(ki);
  Serial.print(" ");
  Serial.println(kd);
  delay(1000); 
}
