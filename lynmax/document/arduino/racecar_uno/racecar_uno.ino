/*
Developer: KaiChun, Wu 
Email: kai.wu013@gmail.com

racecar software for uno.
work for base controller.
Implement PID controller for speed control.


connect
Hw RPM Sensor digital pin2
ESC           digital pin3
Servo         digital pin9
*/

#include <PID_v1.h>//Arduino-PID-Library
//https://github.com/br3ttb/Arduino-PID-Library.git

#include <Servo.h> 

Servo servo;  // create servo object to control a servo 
Servo motor;  // create servo object to control a motor ESC
double out_1 ; // save command from computer for servo(0~180 degree)
double out_2 ; // save command from computer for motor(rpm)
String check=0 ; // Checksum

int led = 13;   // led,pin 13 relative to one arduino on board led
int reversing_flag=0; // flag for motor/ESC revers mode(0:forward mode, 1:reversing)
int servovalue=90; // final command for servo
double motoradd=0;     // adjust value for motor/ESC command
double motorvalue=1500;// final command for motor/ESC
double motor_rpm=0;    // motor speed (rpm)
double motor_rpm_old1=0;    // motor speed (rpm)
double motor_rpm_old2=0;    // motor speed (rpm)
//static const int decoder_pin = 2;//Interrupt pin for arduino,digital pin2 is Interrupt pin0
//digital pin3 is Interrupt pin1
volatile double decoder_counter = 0;
unsigned long time=0;
unsigned long time2;
unsigned long time3;
unsigned long time4=0;

double Kp=0.002, Ki=0.009, Kd=0.005;
PID myPID(&motor_rpm, &motoradd, &out_2, Kp, Ki, Kd, DIRECT);

void setup() 
{ 
 Serial.begin(57600);
  while (!Serial) {
  }
  Serial.setTimeout(20); // remember to set timeout, default is 1000ms
  servo.attach(9);  // attaches the servo on pin 9 to the servo object 
  motor.attach(3);  // attaches the ESC   on pin 3 to the servo object 
  servo.write(90);  // inital command servo to middle point for safe
  motor.writeMicroseconds(1500); // inital command ESC to middle point for safe
  attachInterrupt(0, decoder_isr , FALLING); // setting the external Interrupt.(for rpm sensor)
  pinMode(led, OUTPUT);
  digitalWrite(led,HIGH);
  myPID.SetMode(AUTOMATIC); //turn the PID on
} 

void loop() {
    if (Serial.available()) {
      String commad_string = Serial.readString(); // alternative: read() or readBytes()
      out_1 = getValue(commad_string, ',', 0).toInt(); 
      out_2 = getValue(commad_string, ',', 1).toInt();
      check= getValue(commad_string, ',', 2);
      if(check.toInt() != (commad_string.length()-2-check.length()) )
      {
        return;
      } 
      
      
     // int out_4 = getValue(commad_string, ',', 3).toInt();
       if(out_1==0 && out_2==0) 
       {
         servovalue=90;
         motorvalue=1500;
       }
       else if  (out_2<0   &&  reversing_flag==0) 
       { //start Reversing mold Initialize settings. 
         servo.write(out_1);   //servo keep original command .    
         
         //Command ESC to middle point.
         motor.writeMicroseconds(1500); 
         time3=millis();
         while ((millis()-time3)<30)
         {
          // Serial.println("re2");
          motor.writeMicroseconds(1500);
         }
         
         //Command ESC to backword area(in this time is command ESC to break).
         motor.writeMicroseconds(1700);
         time3=millis();
         while ((millis()-time3)<150)
         {
           motor.writeMicroseconds(1700);
         }
         
         //Command ESC back to middle point.
         motor.writeMicroseconds(1500);
         time3=millis();
         while ((millis()-time3)<200)
         {
          motor.writeMicroseconds(1500);
         }
         
         //Command ESC a large backword value to start up.
         motor.writeMicroseconds(1630);
         time3=millis();
         while ((millis()-time3)<200)
         {
          motor.writeMicroseconds(1630);
         }         
         
         servovalue=out_1;
         myPID.Compute();
         motorvalue=1570-motoradd;
         reversing_flag=1;
       }
       else if (out_2<0   &&  reversing_flag==1)
       {
         servovalue=out_1;
         myPID.Compute();
         motorvalue=1570-motoradd;
       }
       else if (out_2>0)
       {
         servovalue=out_1;
         myPID.Compute();
         motorvalue=1460-motoradd;
         reversing_flag=0;
       }
       else 
       {
         servovalue=out_1;
         motorvalue=1500;
       reversing_flag=0;
       }
       time=millis();
      //Serial.println(out_1);
    }



    if (reversing_flag==1)
    {
      digitalWrite(led,LOW);
    }
    else
    {
      digitalWrite(led,HIGH);
    }
    
    time2=millis();
    if ((time2-time)>1000){
      servovalue=90;
      motorvalue=1500;
    }
    
    
    if ((millis()-time4)>=100)
    {
     motor_rpm=decoder_counter*1000/(millis()-time4)/2*60 ;//rpm
     if (reversing_flag==1)
     {
       motor_rpm=-motor_rpm;
     }
    
     motor_rpm=(motor_rpm_old1+motor_rpm_old2+motor_rpm)/3;
     motor_rpm_old2=motor_rpm_old1;
     motor_rpm_old1=motor_rpm;
     decoder_counter=0;
     time4=millis();
     char str[20];
     sprintf(str, "%d", motor_rpm  );
     Serial.println(motor_rpm);
   } 
   servo.write(servovalue);
   motor.writeMicroseconds(motorvalue);
}
    
void decoder_isr() {
  decoder_counter += 1;
}
    
    
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
