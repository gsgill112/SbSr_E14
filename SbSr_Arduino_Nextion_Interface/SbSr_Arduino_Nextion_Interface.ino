/*

Ricardo Mena C
ricardo@crcibernetica.com
http://crcibernetica.com

  This example code is in public domain

*/

#include <SoftwareSerial.h>
#include "Timer.h"
#include <Nextion.h>

SoftwareSerial nextion(2, 3);// Nextion TX to pin 2 and RX to pin 3 of Arduino

Nextion myNextion(nextion, 9600); //create a Nextion object named myNextion using the nextion serial port @ 9600bps
Timer t;

void setup() {
  Serial.begin(9600);
  myNextion.init();
  pinMode(4,OUTPUT);     // Display Boot Successful
  digitalWrite(4,LOW);
  pinMode(5,OUTPUT);     // Ride Screen     
  digitalWrite(5,LOW);
  pinMode(6,OUTPUT);     // Bump Pressed
  digitalWrite(6,LOW);
  pinMode(7,OUTPUT);     // Self Check Screen
  digitalWrite(7,LOW);
  pinMode(8,OUTPUT);     // Music Screen
  digitalWrite(8,LOW);
  pinMode(9,OUTPUT);     // Music Left
  digitalWrite(9,LOW);
  pinMode(10,OUTPUT);     // Music Play Pause
  digitalWrite(10,LOW);
  pinMode(11,OUTPUT);     // Music Right 
  digitalWrite(11,LOW);
  t.every(1000, resetAll);
}

void loop() {
  String message = myNextion.listen(); //check for message
  if(message != ""){ // if a message is received...

   if( message == "6"){
    Serial.println("Booted");
    digitalWrite(6,HIGH);
   }else if (message == '0xFF'){
     Serial.println("Off");
     digitalWrite(4,LOW);
    // Ride Screen  
    digitalWrite(5,LOW);
    // Options Screen
    digitalWrite(6,LOW);
    // Self Check Screen
    digitalWrite(7,LOW);
    // Music Screen
    digitalWrite(8,LOW);
    // Music Left
    digitalWrite(9,LOW);
    // Music Play Pause
    digitalWrite(10,LOW);
    // Music Right 
    digitalWrite(11,LOW);
   }else if (message == "7"){
     Serial.println("Riding Screen");
     digitalWrite(5,HIGH);
   }else if (message == "65 7 6 0 ffff ffff ffff"){
     Serial.println("Bump Pressed");
     digitalWrite(6,HIGH);
   }else if (message == "8"){
     Serial.println("Music Controls");
     digitalWrite(8,HIGH);
   }else if (message == "65 8 4 0 ffff ffff ffff"){
     Serial.println("Left Button");
     digitalWrite(9,HIGH);
   }else if (message == "65 8 5 0 ffff ffff ffff"){
     Serial.println("Play Pause Button");
     digitalWrite(10,HIGH);
   }else if (message == "65 8 3 0 ffff ffff ffff"){
     Serial.println("Right Button");
     digitalWrite(11,HIGH);
   }else if (message == "3"){
     Serial.println("Self Check Page");
     digitalWrite(7,HIGH);
   }else {
    Serial.println("INVALID INPUT");
    digitalWrite(4,LOW);
    // Ride Screen  
    digitalWrite(5,LOW);
    // Options Screen
    digitalWrite(6,LOW);
    // Self Check Screen
    digitalWrite(7,LOW);
    // Music Screen
    digitalWrite(8,LOW);
    // Music Left
    digitalWrite(9,LOW);
    // Music Play Pause
    digitalWrite(10,LOW);
    // Music Right 
    digitalWrite(11,LOW);
   }
    Serial.println(message); //...print it out
  }

}

void resetAll(void){
    digitalWrite(4,LOW);
    // Ride Screen  
    digitalWrite(5,LOW);
    // Options Screen
    digitalWrite(6,LOW);
    // Self Check Screen
    digitalWrite(7,LOW);
    // Music Screen
    digitalWrite(8,LOW);
    // Music Left
    digitalWrite(9,LOW);
    // Music Play Pause
    digitalWrite(10,LOW);
    // Music Right 
    digitalWrite(11,LOW);
  Serial.println("Reset All Lines");
  }
  
