/*
Written by Jesse Merritt www.github.com/jes1510 October 5, 2013

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/

Firmware for driving a robot arm that will load disks into a DVD or CD tray.

You can assume that I have no idea what I am doing and therefor am not responsible for any damage or negative consequences for using anything from this project. It may destroy all of your equipment and burn down your house.


PD4    6    Motor
PD5    11    Motor

*/
//http://www.vanadiumlabs.com/mini.html
//http://www.vanadiumlabs.com/docs/mini-manual.pdf

#include <Servo.h> 

#define VERSION 0.1

#define  ON       1
#define  OFF      0

#define  BASEA    4
#define  BASEB    6
#define  PUMPA    5
#define  PUMPB    7
#define  ARM_UP_POSITION  26
#define  ARM_DN_POSITION  75      // Was 150
#define  VENTPIN  14
#define  ENCODER_PIN  2
#define  ENCODER_MAX  64
#define  LEFT_LIMIT_PIN   3
#define  ACTUATOR_LIMIT  16
#define  WRIST_RAISED_POSITION  45
#define  WRIST_TOP_POSITION  105
#define  WRIST_DN_POSITION  110
#define  MAX_RETRIES      5
#define  BASE_CENTER_POSITION 28

#define  SHOULDER_PIN    9
#define  ELBOW_PIN      10
#define  WRIST_PIN      11

volatile int encoder_counter;
long debounce_time  = 1;    // mS to debounce
volatile unsigned long last_encoder_time;
volatile unsigned long last_limit_time;



volatile int current_location = 0;
volatile bool pump_state = OFF;
volatile bool vent_state = OFF;
volatile int arm_position = ARM_UP_POSITION;
volatile int wrist_position = WRIST_RAISED_POSITION;
volatile bool debug_mode = false;
int retry_counter = 0;
volatile bool echo = true;

char inByte;
String inData = ""; 
Servo shoulder;  // create servo object to control a servo 
Servo elbow;
Servo wrist; 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  shoulder.attach(SHOULDER_PIN);  // attaches the servo on pin 9 to the servo object 
  elbow.attach(ELBOW_PIN);
  wrist.attach(WRIST_PIN);
  pinMode(BASEA, OUTPUT);
  pinMode(BASEB, OUTPUT);
  pinMode(PUMPA, OUTPUT);
  pinMode(PUMPB, OUTPUT);
  pinMode(VENTPIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT);
  pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);
  pinMode(ACTUATOR_LIMIT, INPUT_PULLUP);
  
  attachInterrupt(0, encoder_debounce, CHANGE);
  attachInterrupt(1, limit_debounce, FALLING);
  
  Serial.begin(9600);
  Serial.println("ready\n");
 // wrist.write(90);
 lift_wrist();
  arm_up();
  delay(500);
  left_home();
  if (echo) prompt();
  
} 
 
 
void loop() 
{ 
  
  while (Serial.available())
  {    
    
    inByte = (char)Serial.read();   
    if (echo) Serial.print(inByte);
    
    if (inByte == '\r')        // Once a /r is recvd check to see if it's a good command
    {     
      if (echo) Serial.println();      
      if (inData == "up") lift_arm();
      else if (inData == "downs")       down_seek(50, 0);   
      else if (inData == "down")        down(50, ARM_DN_POSITION);    
      else if (inData == "pump on")     pump(ON);    
      else if (inData == "pump off")    pump(OFF);  
      else if (inData == "vent")        pump_release();    
      else if (inData == "left")        baseLeft(16);  
      else if (inData == "right")       baseRight(BASE_CENTER_POSITION);   
      else if (inData == "stop")        baseStop();  
      else if (inData == "home")        left_home();       
      else if (inData == "status")      stats();   
      else if (inData == "downseek")    down_seek(50, 1);
      else if (inData == "get disk")    get_disk();
      else if (inData == "load disk")   load_disk();
      else if (inData == "place disk")  place_disk();
      else if (inData == "debug on")    debug_mode = true;
      else if (inData == "debug off")   debug_mode = false;   
      else if (inData == "echo on")     echo = true;
      else if (inData == "echo off")     echo = false;
      
      
      else 
      {
        Serial.print("Error, Unknown Command: ");
        Serial.println(inData);
      }
      inData = "";       // Clear the buffer 
      if (echo) prompt();

    }
    else inData += inByte;  // If it's not a /r then keep adding to the string
  }  

} 

void prompt()
{

  Serial.print("Burner_V");
  Serial.print(VERSION);
  Serial.print("> ");
}
void place_disk()
{
  lift_wrist();
  down(50, 40);
  delay(1000);
  shoulder.write(arm_position + 15);
  delay(2000);
  pump_release();
}
  
  
  
void load_disk(void)
{
  if (digitalRead(ACTUATOR_LIMIT) == HIGH)   
    {
      delay(50);   
      if (digitalRead(ACTUATOR_LIMIT) == HIGH) 
      {
        get_disk();
      }
    }
  
  baseRight(BASE_CENTER_POSITION); 
  delay(500);
//  pump_release();
}


void reportError(String message)
{
  Serial.print("Error: ");
  Serial.println(message);
}

void get_disk()
{
  int try_counter = 0;
  left_home();
  delay(500);
  down_seek(50, 1);
  pump(ON);
  delay(2000);
  lift_arm();
  delay(500);

  if (digitalRead(ACTUATOR_LIMIT) == HIGH)   
    {
      delay(50);   
      if (digitalRead(ACTUATOR_LIMIT) == HIGH) 
      {
        get_disk();
        retry_counter++;
        if (retry_counter > MAX_RETRIES)
        {
          reportError("No Disk");
          exit;
        }
      }
    }
}

// Moves the arm down in a simple move.  Doesn't sync wrist
void down(int delay_time, int p)
{

  for(pos = arm_position; pos < p; pos += 1)  // We start up and increment down
  {                                  
   // wrist_position = map(pos, 26, 150, 100, 105);    // Remap the wrist end points to something sane for the arm
    shoulder.write(pos);              // Write it to the servo    
    elbow.write(pos);                 // Write it to the servo
    //wrist.write(wrist_position);      // Write it to the servo
    delay(delay_time);                // Slow down the loop according to specified time
    arm_position = pos;               // Track the arm position
     

  } 
  Serial.println("OK");    // Report all went well
}



// Move the arm down, keeping wrist level
void down_seek(int delay_time, int use_limit)
{

  for(pos = arm_position; pos < ARM_DN_POSITION; pos += 1)  // We start up and increment down
  {                                  
    wrist_position = map(pos, 26, 150, 100, 105);    // Remap the wrist end points to something sane for the arm
    shoulder.write(pos);              // Write it to the servo    
    elbow.write(pos);                 // Write it to the servo
    wrist.write(wrist_position);      // Write it to the servo
    delay(delay_time);                // Slow down the loop according to specified time
    arm_position = pos;               // Track the arm position
    if ((digitalRead(ACTUATOR_LIMIT) == LOW) && use_limit)  // If using the limit then look for the switch activation
    {
      delay(50);   // Wait a bit for debounce
      if (digitalRead(ACTUATOR_LIMIT) == LOW) // If it's still low then assume it's a good trigger and we're at the bottom
      {
        shoulder.write(pos + 5);    // Move down a bit more to make sure it's seated on the disk
        elbow.write(pos + 5);
        arm_position = pos + 5;
           
        break;    // Bail out of the loop since we're at the end
      }
      
    }     

  } 
  Serial.println("OK");    // Report all went well
}

//    Shows the current status and positions
void stats(void)
{
  Serial.print("Location (Counts): ");
  Serial.println(current_location);
  Serial.print("Pump: ");
  Serial.println(pump_state);
  Serial.print("Vent: ");
  Serial.println(vent_state);
  Serial.print("ARM Angle: ");
  Serial.println(arm_position);
  Serial.print("WRIST Angle: ");
  Serial.println(wrist_position);
  Serial.print("DEBUG: ");
  Serial.println(debug_mode);
}

//    Return to the home position
void left_home(void)
{
  encoder_counter = 0;  // Reset the location
  lift_arm();
  // int h = digitalRead(LEFT_LIMIT);  //; init the switch variable
  current_location = ENCODER_MAX;              // Set to the MAX. THe home INT will set it to 0;
  if (debug_mode) Serial.println("HOMING");
  
  if (digitalRead(LEFT_LIMIT_PIN) == LOW) 
  {
    delay(20);
    if (digitalRead(LEFT_LIMIT_PIN) == LOW) 
    {
      current_location = 0;
    }
  }
    
  while (current_location > 0)  // If it's not triggered then move the arm
  {
    
    digitalWrite(BASEA, LOW);    // Moves the arm left
    digitalWrite(BASEB, HIGH);  
  }
  baseStop();  // The stop the arm  
 
  Serial.println("OK");  // Report all is ok
}
  


// Debounce the readings from the encoder wheel
void encoder_debounce(void)
{
 
  if ((long) (micros() - last_encoder_time) >= debounce_time * 1000)  // If the microseconds is greater than the debounce time
  {
    encoder_Interrupt();    // trigger the interrupt
    last_encoder_time = micros();    // Reset the timer    
  }
}

void limit_Interrupt(void)
{
  
  current_location = 0;
  encoder_counter = 0;
  if(debug_mode) Serial.println("HOME!");  
}

void limit_debounce(void)
{ 
  
  if ((long) (micros() - last_limit_time) >= 20 * 1000)  // If the microseconds is greater than the debounce time
  {
    limit_Interrupt();    // trigger the interrupt
    last_limit_time = micros();    // Reset the timer    
  }
}



// Activate or deactivate the pump
void pump(int state)
{
  noInterrupts();            // Disable interrupts
  digitalWrite(PUMPA, LOW);  // Pump only runs in one direction
  digitalWrite(PUMPB, state);    
  Serial.println("OK");      // Report all is ok
  pump_state = state;        // set the state
  delay(500);
  interrupts();              // Reenable interrupts

}
 
 
// Vent the air line and turn off pump
void pump_release(void)
{
  noInterrupts();                // Disable interrupts.  This operation can be noisy
  pump(0);                      // Stop the pump
  digitalWrite(VENTPIN, HIGH);  // Turn on the vent solenoid
  delay(1000);                  // Wait a second
  digitalWrite(VENTPIN, LOW);  // Turn off the vent solenoid
  Serial.println("OK");         // REport all is ok
  delay(500);
  interrupts();                // reenable interrupts
 
}
  
  
void baseRight(int steps)
{
  digitalWrite(BASEA, HIGH);
  digitalWrite(BASEB, LOW);
  encoder_counter = 0;  
  while(encoder_counter < steps)
  {
    
  }
  baseStop();
  current_location += encoder_counter;
  Serial.println("OK");

}

void baseStop(void)
{
  digitalWrite(BASEA, LOW);
  digitalWrite(BASEB, LOW);

}

void baseLeft(int steps)
{
  digitalWrite(BASEA, LOW);
  digitalWrite(BASEB, HIGH);
  encoder_counter = 0;
  while(encoder_counter < steps)
  {
    //Serial.println(encoder_Counter);
  }
  baseStop();
  current_location -= encoder_counter;
  Serial.println("OK");

}


void arm_up(void)
{
  shoulder.write(ARM_UP_POSITION);
  elbow.write(ARM_UP_POSITION);
  Serial.println("OK");
}

void arm_down(void)
{
  shoulder.write(ARM_DN_POSITION);
  elbow.write(ARM_DN_POSITION);
  Serial.println("OK");
}

void lift_arm(void)
{    

  for(pos = arm_position; pos>=ARM_UP_POSITION; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    shoulder.write(pos);              // tell servo to go to position in variable 'pos' 
    elbow.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position 
    arm_position = pos;
    
  } 
  lift_wrist();
  Serial.println("OK");

}

void lift_wrist(void)
{
  for(pos=wrist_position; pos>WRIST_RAISED_POSITION; pos-=1)
  {
    wrist.write(pos);
    delay(15);
  }
}

void lower_wrist(void)
{
  // Was 105
  for(pos=45; pos<100; pos++)
  {
    wrist.write(pos);
    delay(15);
  }
}
  


void encoder_Interrupt(void)
{
  encoder_counter++;
  if (debug_mode) Serial.println(encoder_counter);
  if(encoder_counter > ENCODER_MAX) encoder_counter = 0;    // Reset the counter   
  
  
  
}