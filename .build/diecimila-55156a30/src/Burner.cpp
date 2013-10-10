#include <Arduino.h>
#include <Arduino.h>
#include <Servo.h> 
#include "ISR.h"
#include "platform.h"
void setup();
void loop();
void prompt();
void place_disk();
void load_disk(void);
void reportError(String message);
void get_disk();
void down(int delay_time, int p);
void down_seek(int delay_time, int use_limit);
void stats(void);
void left_home(void);
void pump(int state);
void pump_release(void);
void baseRight(int steps);
void baseStop(void);
void baseLeft(int steps);
void arm_up(void);
void arm_down(void);
void lift_arm(void);
void lift_wrist(void);
void lower_wrist(void);
#line 1 "src/Burner.ino"
/*
Written by Jesse Merritt www.github.com/jes1510 October 5, 2013

This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the 
Free Software Foundation, either version 3 of the License, or (at your
option) any later version.

This program is distributed in the hope that it will be useful, but 
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
for more details.

You should have received a copy of the GNU General Public License along 
with this program. If not, see <http://www.gnu.org/licenses/

Firmware for driving a robot arm that will load disks into a DVD or CD tray.

You can assume that I have no idea what I am doing and therefor am not 
responsible for any damage or negative consequences for using anything
from this project. It may destroy all of your equipment and burn down your house.
 
http://www.vanadiumlabs.com/mini.html
http://www.vanadiumlabs.com/docs/mini-manual.pdf

Connections
		2	Encoder
		3	Limit switch
PD4    	6	Shoulder Motor	
PD5    11	Shoulder Motor
PD5		5	Pump
PD7		7	Pump
* 		16	Actuator Limit
* 		9	Shoulder servo
* 		10	Elbow servo	
* 		11	Wrist Servo

*/

//#include <Arduino.h>
//#include <Servo.h> 
//#include "ISR.h"
//#include "platform.h"


int current_location = ENCODER_MAX;		// Tracks location of shoulder.  Preload it with the max value
int encoder_counter = 0;		// Raw counter for encoder
 
bool pump_state = OFF;			// Pump state machine
bool vent_state = OFF;			// Vent state machine
int arm_position = ARM_UP_POSITION;		// arm state tracker
int wrist_position = WRIST_RAISED_POSITION;	// wrist state tracker
bool debug_mode = false;		// debug or verbosity state machine
int retry_counter = 0;			// Number of times to retry on failure
bool echo = true;				// Echo state machine for interactive mode

unsigned long last_encoder_time = 0;	// Debounce counter for encoder
unsigned long last_limit_time = 0;		// Debounce countrer for limit switch




char inByte;		// byte coming in serial port
String inData = ""; 	// String for serial data 
Servo shoulder;  // servo object for shoulder
Servo elbow;	// servo object for elbow
Servo wrist; 	// servo object for wrist
int pos = 0;    // variable to store the servo position 

 
void setup() 
{ 
	shoulder.attach(SHOULDER_PIN);  // Connect shoulder servo object
	elbow.attach(ELBOW_PIN);		// connect elbow servo object
	wrist.attach(WRIST_PIN);		// Connect elbow servo object
	pinMode(BASEA, OUTPUT);			// Drive motor on base is output
	pinMode(BASEB, OUTPUT);			// Other pin of drive motor on base
	pinMode(PUMPA, OUTPUT);			// Pump is driven by output
	pinMode(PUMPB, OUTPUT);			// Other pin of pump
	pinMode(VENTPIN, OUTPUT);		// Vent solenoid is single ended output
	pinMode(ENCODER_PIN, INPUT);	// Input for encoder disk on base
	pinMode(LEFT_LIMIT_PIN, INPUT);	// Input of limit switch on base (home)
	pinMode(ACTUATOR_LIMIT, INPUT);	// Actuator limit switch to detect disks

	digitalWrite(LEFT_LIMIT_PIN, HIGH);	// Turn on pullup
	digitalWrite(ACTUATOR_LIMIT, HIGH);	// Turn on pullup

	attachInterrupt(0, encoder_debounce, CHANGE);	// Trigger interrupt on state change
	attachInterrupt(1, limit_debounce, FALLING);	// trigger interrupt on low

	Serial.begin(9600);				// 9600 baud, 8N1
	Serial.println("ready\n");		// Notify host that arm is ready for command
	lift_wrist();					// Lift the wrist to top position
	arm_up();						// Lift the arm to the top position
	delay(500);						// Give the servos time to settle
	if (digitalRead(LEFT_LIMIT_PIN) == LOW) current_location = 0;
	left_home();					// Move the arm to the home position
	if (echo) prompt();  			// If the echo is on then print the prompt	
	
} 


 
void loop() 
{ 

  while (Serial.available())		// If serial data is available on the port
  {    		
    inByte = (char)Serial.read();  		// Read a byte 
    if (echo) Serial.print(inByte);		// If the scho is on then print it back to the host
    
    if (inByte == '\r')        // Once a /r is recvd check to see if it's a good command
    {     
      if (echo) Serial.println();   // Dummy empty line   
      if (inData == "up") lift_arm();		
      else if (inData == "downs")       down_seek(50, 0);  		// Lower arm until it finds bottom 
      else if (inData == "down")        down(50, ARM_DN_POSITION); 	// Blindly lower arm
      else if (inData == "pump on")     pump(ON);    				// Start pump
      else if (inData == "pump off")    pump(OFF);  				// Stop pump
      else if (inData == "vent")        pump_release();    		// Stop pump and vent the vacuum
      else if (inData == "left")        baseLeft(16);  			// Move base to the left
      else if (inData == "right")       baseRight(BASE_CENTER_POSITION);   	// Move base to the right
      else if (inData == "stop")        baseStop();  				// Stop base
      else if (inData == "home")        left_home();       		// Mode base left until it finds home
      else if (inData == "status")      stats();   				// Show current status
      else if (inData == "stats")   	 stats();   					// Show current stats
      else if (inData == "get disk")    get_disk();				// Macro to get disk and lift to top
      else if (inData == "load disk")   load_disk();				// Macro to get disk and move to center
      else if (inData == "place disk")  place_disk();				// Macro to lower disk into tray
      else if (inData == "debug on")    debug_mode = true;			// turn on debug mode, prints encoder values
      else if (inData == "debug off")   debug_mode = false;   		// Turn off debug mode
      else if (inData == "echo on")     echo = true;				// Turn on echo mode
      else if (inData == "echo off")     echo = false;				// Turn off echo mode      
      
      else 			// The string wasn't recognized so it was a bad command
      {	
		Serial.print("Error, Unknown Command: ");		// Show the error
        Serial.println(inData);							// Echo the command 
      }
      inData = "";       	// Clear the buffer (string)
      if (echo) prompt();	// reprint the prompt

    }
    else inData += inByte;  // If it's not a /r then keep adding to the string (buffer)
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
  if (debug_mode) Serial.println("HOMING");
	
  while (current_location > 0)  // If it's not triggered then move the arm
  {   
    digitalWrite(BASEA, LOW);    // Moves the arm left
    digitalWrite(BASEB, HIGH);  
  }
  baseStop();  // Stop the arm  
 
  Serial.println("OK");  // Report all is ok
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
  noInterrupts();               // Disable interrupts.  This operation can be noisy
  pump(0);                      // Stop the pump
  digitalWrite(VENTPIN, HIGH);  // Turn on the vent solenoid
  delay(1000);                  // Wait a second
  digitalWrite(VENTPIN, LOW); 	// Turn off the vent solenoid
  Serial.println("OK");         // Report all is ok
  delay(500);
  interrupts();                // reenable interrupts
 
}
  
  
void baseRight(int steps)
{
  digitalWrite(BASEA, HIGH);
  digitalWrite(BASEB, LOW);
  //encoder_counter = 0;  
  while(encoder_counter < steps)
  {
   delay(1);
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



