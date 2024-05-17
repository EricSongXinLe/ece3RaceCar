// Base code.
// 
// *  NOTE: this code will do only three things:
// *    --rotate one wheel, and 
// *    --blink the right front mainboard LED.
// *    
// *  You will need to add more code to
// *  make the car do anything useful. 
// 
//#include <ECE3_LCD7.h>
#include <ECE3.h>
//uint16_t sensorValues[8]; // right -> left, 0 -> 7
uint16_t sensorValues[8];
const int weight[8] = {-15,-14,-12,-8,8,12,14,15};
int hisError = 0;
int sumError = 0;
int processedValue[8];

const int turn_around_thres = 390;

bool turned = false;
int timeCnt = 0;
float Kp = 0.215;
const float Ki = 0.0005;
float Kd = 0.75;

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int LED_RF = 41;

bool meetblack = false;
///////////////////////////////////
void setup() {
// put your setup code here, to run once:
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);
  ECE3_Init();
  

// set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  resetEncoderCount_left();
  resetEncoderCount_right();
  delay(1000); //Wait 1 second before starting 
  
}

int getEncoderVal(void){
  int L = getEncoderCount_left();
  int R = getEncoderCount_right();
  return (L+R)/2;
}

void turn_around(void){
  bool finishedTurn = false;
  resetEncoderCount_left();
  resetEncoderCount_right();
  analogWrite(left_pwm_pin,0);
  analogWrite(right_pwm_pin,0);
  delay(300);
  analogWrite(left_pwm_pin,35);
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(right_dir_pin,HIGH);
  analogWrite(right_pwm_pin,35);
  while(!finishedTurn){
    if(getEncoderCount_left() > turn_around_thres){
      finishedTurn = true;
    }
  }
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(left_dir_pin,LOW);
  resetEncoderCount_left();
  resetEncoderCount_right();
  analogWrite(right_pwm_pin,35);
  analogWrite(left_pwm_pin,35);
  return;
}

int abs(int input){
  if (input > 0){
    return input;
  }else{
    return 0-input;
  }
}

uint16_t minVal(uint16_t input[],short length){
  uint16_t currMin = 2501;
  for(unsigned char i =0; i<length;i++){
    if(currMin > input[i]){
      currMin = input[i];
    }
  }
  return currMin;
}

uint16_t maxVal(uint16_t input[],short length){
  uint16_t currMax = 0;
  for(unsigned char i =0; i<length;i++){
    if(currMax < input[i]){
      currMax = input[i];
    }
  }
  return currMax;
}

bool finishLine(uint16_t input[],short length){
  char count = 0;
  for(unsigned char i =0; i<length;i++){
    if(input[i] > 1800) {
      //1800 may need to change 
      count += 1;
    }
  }
  if(count >= 7){
    return true;
  }else{
    return false;
  }
}

void setSpd(int left, int right){
  if(left<0){
    digitalWrite(left_dir_pin,HIGH);
    left *= 1.5;
  }else{
    digitalWrite(left_dir_pin,LOW);
  }

  if(right<0){
    digitalWrite(right_dir_pin,HIGH);
    right *= 1.5;
  }else{
    digitalWrite(right_dir_pin,LOW);
  }
  analogWrite(left_pwm_pin,0);
  analogWrite(right_pwm_pin,abs(right));
  analogWrite(left_pwm_pin,abs(left));
  analogWrite(right_pwm_pin,abs(right));
}

void loop() {
  timeCnt++;
  // put your main code here, to run repeatedly: 
  int leftSpd = 30;
  int rightSpd = 30;
//  ECE3_read_IR(sensorValues);
  Kp = 0.215;
  Kd = 0.75;
  if(timeCnt>1250 && timeCnt < 1400){
    Kp = 0.11;
    Kd = 0.5;
  }
  if(timeCnt>3220 && timeCnt < 3350){
    Kp = 0.11;
    Kd = 0.5;
  }
  if(timeCnt == 1400 || timeCnt ==3350){
    digitalWrite(left_dir_pin,LOW);
    digitalWrite(right_dir_pin,LOW);
    analogWrite(left_pwm_pin,40);
      analogWrite(right_pwm_pin,40);
      delay(4700);
  }
// 
  
  ECE3_read_IR(sensorValues);

  uint16_t min = minVal(sensorValues,8);
  if(meetblack&&finishLine(sensorValues,8)){
    // turn 180 around
    if(turned){
      leftSpd = 0;
      rightSpd = 0;
      analogWrite(left_pwm_pin,leftSpd);
      analogWrite(right_pwm_pin,rightSpd);
      delay(10000);
    }
    turn_around();
    turned = true;
    meetblack = false;
    return;
  }else if(finishLine(sensorValues,8)){
    meetblack = true;
  }else{
    meetblack = false;
  }
  //Serial.println(min);

  for(unsigned char i = 0; i < 8; i++){
    sensorValues[i] -= min;
  }

  uint16_t max = maxVal(sensorValues,8);

  for(unsigned char i = 0; i < 8; i++){
    sensorValues[i]=(sensorValues[i]*1000/max);
  }

  for(unsigned char i = 0; i < 8; i++){
    processedValue[i] = sensorValues[i];
  }

  for(unsigned char i = 0; i < 8; i++){
    processedValue[i] *= weight[i];
    processedValue[i] /= 8;
  }

  int error = 0;
  for(unsigned char i = 0; i < 8; i++){
    error+=processedValue[i];
  }
  error /= 8;

  if(abs(error)<15){
    error = 0;
  }
  if(error < 0){
    leftSpd -= Kp*error+Ki*sumError;
    rightSpd += Kp*error+Ki*sumError; 
    // error is negative. - means add and + means -
  }else{
    rightSpd += Kp*error+Ki*sumError;
    leftSpd -= Kp*error+Ki*sumError;
  }
  
  
  
  int Neterror = abs(error);

  int delta_error = Neterror - hisError;
  if(error < 0){
    leftSpd += Kd*delta_error;
    rightSpd -= Kd*delta_error; 
  // error is negative. - means add and + means -
  }else{
    rightSpd += Kd*delta_error;
    leftSpd -= Kd*delta_error;
  }
    
  hisError = Neterror;
  sumError += error;
  if(leftSpd < 0 || rightSpd < 0){
    digitalWrite(LED_RF, HIGH);
  }
  else{
    digitalWrite(LED_RF, LOW);
  }
  setSpd(leftSpd,rightSpd);
  /*
  analogWrite(left_pwm_pin,leftSpd);
  analogWrite(right_pwm_pin,rightSpd);
  */
  
      // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
      // 2500 means minimum reflectance
      /*
      for (unsigned char i = 0; i < 8; i++)
      {
        Serial.print(sensorValues[i]);
        Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
      }
      //      Serial.print(readCount);
      //      Serial.print('\t');
      //      Serial.print(bumpSw2Reading);
      Serial.println();
      delay(1000);*/


    
  }
