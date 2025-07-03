#include <Arduino.h>
#define A_pin 35
#define B_pin 34
int degree=390;
int state;
int state0;
bool flag = false;
void counter();
void change_flag();

void setup()
{
  attachInterrupt(34,change_flag,CHANGE);
  Serial.begin(115200);
}

void loop()
{
  counter();
  while (Serial.available())
  {
    Serial.read();
    Serial.println(degree);
  }
  
}

void counter()
{
  flag = false;
  bool A_result = digitalRead(A_pin);
  bool B_result = digitalRead(B_pin);
  if(A_result == true && B_result == true)
  {
    state=0;
  }
  else if(A_result == false && B_result == true)
  {
    state=1;
  }
   else if (A_result == false && B_result == false)
  {
    state=2;
  }
  else if (A_result == true && B_result == false)
  {
    state=3;
  }
 
  
  if ((state>state0)||(state==0&&state0==3))
  {
    degree+=1;
    degree=degree%779;
    //Serial.println(degree);

  }
  else if((state<state0)||(state==3&&state0==0))
  {
    degree-=1;
    degree=degree%779;
    //Serial.println(degree);
  }
  state0=state;
}

void change_flag()
{
  flag = true;
}