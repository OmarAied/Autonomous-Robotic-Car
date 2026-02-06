/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * V2 Arduino Smart Car Tutorial Lesson 3
 * Tutorial URL http://osoyoo.com/2018/12/18/osoyoo-robot-car-kit-lesson-3-object-follow-robot-car/
 * CopyRight www.osoyoo.com 
 * 
 */
//Define L298N Dual H-Bridge Motor Controller Pins
#define speedPinR 9   // RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1  12    //  Right Motor direction pin 1 to MODEL-X IN1 
#define RightDirectPin2  11    // Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6        //  Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1  7    // Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2  8   ///Left Motor direction pin 1 to MODEL-X IN4
/*From left to right, connect to D3,A1-A3 ,D10*/
#define RightObstacleSensor 2  //Right obstacle sensor to D2 (front direction is from arduino point to voltage meter)
#define LeftObstacleSensor 3   //Left obstacle sensor to D3
#define SPEED   180 //motor in   speed
void go_Advance(void)  //Forward
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
    set_Motorspeed(SPEED,SPEED);
}
void back_Right(void)  //Turn left
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
    set_Motorspeed(SPEED,0);
}
void back_Left(void)  //Turn right
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
    set_Motorspeed(0,SPEED);
}
void go_Back(void)  //Reverse
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
    set_Motorspeed(SPEED,SPEED);
}
void stop_Stop()    //Stop
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
}
/*set motor speed */
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);   
}

void setup()
{
 pinMode(speedPinL,OUTPUT); //left motor PWM pin
 pinMode(speedPinR,OUTPUT); //rignt motor PWM  pin
 pinMode(RightDirectPin1,OUTPUT); //left motor direction pin1
 pinMode(RightDirectPin2,OUTPUT); //left motor direction pin2
 pinMode(LeftDirectPin1,OUTPUT); //right motor direction Pin 1
 pinMode(LeftDirectPin2,OUTPUT);  //right motor direction Pin 2

  /*line follow sensors */
 pinMode(RightObstacleSensor,INPUT); 
  pinMode(LeftObstacleSensor,INPUT); 
 Serial.begin(9600);
}
void auto_following(){
 int IRvalueLeft= digitalRead(RightObstacleSensor);
  int IRvalueRight=digitalRead(LeftObstacleSensor);
 if (IRvalueLeft==LOW && IRvalueRight==LOW)
 { 
  //both sensor detected obstacle, go ahead
      go_Back();  //Turn left
 }
 else  if (IRvalueLeft==HIGH && IRvalueRight==HIGH)
 {
     stop_Stop();   //no obstacle, stop
     set_Motorspeed(0,0);
    }
 else if (IRvalueLeft==LOW && IRvalueRight==HIGH)
 { 
  //only left sensor detect obstacle
      back_Left();  //Turn left
 }
  else if (IRvalueLeft==HIGH && IRvalueRight==LOW)
 { 
   //only right sensor detect obstacle
      back_Right();  //Turn right
  
 }
}
void loop(){
auto_following();
}
