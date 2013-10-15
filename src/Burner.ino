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
		2	Base location encoder
		3	Actuator Limit switch
PD4		6	Shoulder Motor H Bridge side A
PD5		11	Shoulder Motor H Bridge side B
PD5		5	Pump H Bridge side A
PD7		7	Pump H bridge side B
*		16	Actuator Limit
*		9	Shoulder servo
*		10	Elbow servo 
*		11	Wrist Servo

*/

#include <Arduino.h>
#include <Servo.h> 
#include "ISR.h"
#include "platform.h"
 
// State machine inits
bool pump_state = OFF;					// Pump state machine
bool vent_state = OFF;					// Vent state machine
bool debug_mode = false;				// debug or verbosity state machine
bool echo = true;						// Echo state machine for interactive mode

// Serial port and servos
char inByte;				// byte coming in serial port
String inData = "";		 	// String for serial data 
Servo shoulder;				// servo object for shoulder
Servo elbow;				// servo object for elbow
Servo wrist;				// servo object for wrist

// Position trackers
int retry_counter = 0;							// Number of times to retry on failure
int arm_position = ARM_UP_POSITION;		 		// arm state tracker
int wrist_position = WRIST_RAISED_POSITION; 	// wrist state tracker
 
void setup() 
{ 
	shoulder.attach(SHOULDER_PIN);		// Connect shoulder servo object
	elbow.attach(ELBOW_PIN);			// connect elbow servo object
	wrist.attach(WRIST_PIN);			// Connect elbow servo object
	pinMode(BASEA, OUTPUT);				 // Drive motor on base is output
	pinMode(BASEB, OUTPUT);				 // Other pin of drive motor on base
	pinMode(PUMPA, OUTPUT);				 // Pump is driven by output
	pinMode(PUMPB, OUTPUT);				 // Other pin of pump
	pinMode(VENTPIN, OUTPUT);			 // Vent solenoid is single ended output
	pinMode(ENCODER_PIN, INPUT);		// Input for encoder disk on base
	pinMode(LEFT_LIMIT_PIN, INPUT); // Input of limit switch on base (home)
	pinMode(ACTUATOR_LIMIT, INPUT); // Actuator limit switch to detect disks

	digitalWrite(LEFT_LIMIT_PIN, HIGH); // Turn on pullup
	digitalWrite(ACTUATOR_LIMIT, HIGH); // Turn on pullup

	attachInterrupt(0, encoder_debounce, CHANGE);	 // Trigger interrupt on state change
	attachInterrupt(1, limit_debounce, FALLING);	// trigger interrupt on low

	Serial.begin(9600);						 // 9600 baud, 8N1
	Serial.println("ready\n");			// Notify host that arm is ready for command
	lift_wrist(WRIST_RAISED_POSITION);			// Lift the wrist to top position
	arm_up();									// Lift the arm to the top position
	delay(500);									// Give the servos time to settle
	if (digitalRead(LEFT_LIMIT_PIN) == LOW) current_location = 0;	// Check the location on startup
	left_home();				// Move the arm to the home position
	if (echo) prompt();			// If the echo is on then print the prompt			
} 


 
void loop() 
{ 
	while (Serial.available())				// If serial data is available on the port
	{				 
		inByte = (char)Serial.read();			 // Read a byte 
		if (echo) Serial.print(inByte);		 // If the scho is on then print it back to the host
		
		if (inByte == '\r')		 // Once a /r is recvd check to see if it's a good command
		{		
			if (echo) Serial.println();	 // Dummy empty line	 
			if (inData == "up") lift_arm();			 
			else if (inData == "downs")			down_seek(50, 0);				// Lower arm until it finds bottom 
			else if (inData == "down")				down(50, ARM_DN_POSITION);		// Blindly lower arm
			else if (inData == "pump on")			pump(ON);						// Start pump
			else if (inData == "pump off")			pump(OFF);						// Stop pump
			else if (inData == "vent")				pump_release();				 	// Stop pump and vent the vacuum
			else if (inData == "left")				baseLeft(16);					// Move base to the left
			else if (inData == "center")			findCenter();		 			// Move base to the right
			else if (inData == "right")			baseRight(BASE_CENTER_POSITION);// Move base to the right
			else if (inData == "stop")				baseStop();						// Stop base
			else if (inData == "home")				left_home();					// Mode base left until it finds home
			else if (inData == "status")			stats();						// Show current status
			else if (inData == "stats")			stats();						// Show current stats
			else if (inData == "get disk")			get_disk();						// Macro to get disk and lift to top
			else if (inData == "load disk")	 	load_disk();					// Macro to get disk and move to center
			else if (inData == "unload disk")	 	unload_disk();					// Macro to get disk and move to center
			else if (inData == "place disk")		place_disk();					// Macro to lower disk into tray
			else if (inData == "debug on")			debug_mode = true;				// turn on debug mode, prints encoder values
			else if (inData == "debug off")	 	debug_mode = false;				// Turn off debug mode
			else if (inData == "echo on")			echo = true;					// Turn on echo mode
			else if (inData == "echo off")		 	echo = false;					// Turn off echo mode		 
			
			else					// The string wasn't recognized so it was a bad command
			{ 
				Serial.print("Error, Unknown Command: ");			 // Show the error
				Serial.println(inData);												 // Echo the command 
			}
			inData = "";			// Clear the buffer (string)
			if (echo) prompt();	 // reprint the prompt

		}
		else inData += inByte;	// If it's not a /r then keep adding to the string (buffer)
	}	

} 

// simple prompt showing version
void prompt()
{
	Serial.print("Burner_V");
	Serial.print(VERSION);
	Serial.print("> ");
}


void unload_disk()
{
	findCenter();
	int i;
	lower_wrist(wrist_position + 40);
	down(50, 60);
	delay(1000);	
	for (i=0; i<10; i++)
	{
		shoulder.write(arm_position + i);
		delay(50);
	}
	
	pump(1);
	delay(500);	
	down_seek(50, 1);	
	delay(1000);
	lift_wrist(wrist_position - 40);
	delay(1000);
	lift_arm();
	
}

// lower a disk into the tray
void place_disk()
{
	int i;
	lower_wrist(wrist_position + 40);
	down(50, 60);
	delay(1000);	
	for (i=0; i<10; i++)
	{
		shoulder.write(arm_position + i);
		delay(50);
	}
	
	delay(750);
	pump_release();
	lift_arm();
	//left_home();
}
	
	
// Go home and retrieve disk then move to center
void load_disk(void)
{
	if (digitalRead(ACTUATOR_LIMIT) == HIGH)			// Check if a disk is already on the actuator
		{
			delay(50);																// debounce
			if (digitalRead(ACTUATOR_LIMIT) == HIGH) 
			{
				get_disk();				 // No disk detected so we load one
			}
		}
	
	findCenter();
	delay(500);
	place_disk();
}

// Unified function to send error over serial port
void reportError(String message)
{
	Serial.print("Error: ");
	Serial.println(message);
}

// Go home and retrieve disk
void get_disk()
{
	int try_counter = 0;
	left_home();
	delay(1000);
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

// Moves the arm down in a simple move.	Doesn't sync wrist
void down(int delay_time, int p)
{
	int pos;
	for(pos = arm_position; pos < p; pos += 1)	// We start up and increment down
	{															 
	 // wrist_position = map(pos, 26, 150, 100, 105); // Remap the wrist end points to something sane for the arm
		shoulder.write(pos);							// Write it to the servo	
		elbow.write(pos);								// Write it to the servo
		//wrist.write(wrist_position);		// Write it to the servo
		delay(delay_time);							// Slow down the loop according to specified time
		arm_position = pos;						// Track the arm position

	} 
	Serial.println("OK"); // Report all went well
}

// Move the arm down, keeping wrist level
void down_seek(int delay_time, int use_limit)
{
	int pos;
	for(pos = arm_position; pos < ARM_DN_POSITION; pos += 1)	// We start up and increment down
	{															 
		wrist_position = map(pos, 26, 150, 100, 105);	 // Remap the wrist end points to something sane for the arm
		shoulder.write(pos);							// Write it to the servo	
		elbow.write(pos);								// Write it to the servo
		wrist.write(wrist_position);			// Write it to the servo
		delay(delay_time);							// Slow down the loop according to specified time
		arm_position = pos;						// Track the arm position
		if ((digitalRead(ACTUATOR_LIMIT) == LOW) && use_limit)	// If using the limit then look for the switch activation
		{
			delay(50);	 // Wait a bit for debounce
			if (digitalRead(ACTUATOR_LIMIT) == LOW) // If it's still low then assume it's a good trigger and we're at the bottom
			{
				shoulder.write(pos + 5);		// Move down a bit more to make sure it's seated on the disk
				elbow.write(pos + 5);
				arm_position = pos + 5;
					 
				break;	// Bail out of the loop since we're at the end
			}			
		}
	} 
	Serial.println("OK"); // Report all went well
}

//	Shows the current status and positions
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

//	Return to the home position
void left_home(void)
{
	lift_wrist(WRIST_RAISED_POSITION);
	delay(500);
	
	encoder_counter = 0;	// Reset the location
	lift_arm();
	if (debug_mode) Serial.println("HOMING");
		
	while (current_location > 0)	// If it's not triggered then move the arm
	{	 
		digitalWrite(BASEA, LOW);	 // Moves the arm left
		digitalWrite(BASEB, HIGH);	
	}
	delay(500);			// Wait just a bit to reduce bounce
	baseStop();	// Stop the arm 
	Serial.println("OK");	// Report all is ok
}



// Activate or deactivate the pump
void pump(int state)
{
	noInterrupts();					 // Disable interrupts
	digitalWrite(PUMPA, LOW);	// Pump only runs in one direction
	digitalWrite(PUMPB, state);	 
	Serial.println("OK");	 // Report all is ok
	pump_state = state;			 // set the state
	delay(500);
	interrupts();					 // Reenable interrupt
}
 
 
// Vent the air line and turn off pump
void pump_release(void)
{
	noInterrupts();							// Disable interrupts.	This operation can be noisy
	pump(0);										// Stop the pump
	digitalWrite(VENTPIN, HIGH);	// Turn on the vent solenoid
	delay(1000);								// Wait a second
	digitalWrite(VENTPIN, LOW);	 // Turn off the vent solenoid
	Serial.println("OK");			// Report all is ok
	delay(500);
	interrupts();						 // reenable interrupts
 
}


// Move to the center from current location	
void findCenter()
{
	if (current_location > BASE_CENTER_POSITION)
	{
		Serial.println(current_location - BASE_CENTER_POSITION);
		baseLeft(current_location - BASE_CENTER_POSITION);		
	}
	
	if (current_location < BASE_CENTER_POSITION)
	{
		baseRight(BASE_CENTER_POSITION - current_location);
	}
}
	
	
// Move the base right n steps 
void baseRight(int steps)
{
	digitalWrite(BASEA, HIGH);
	digitalWrite(BASEB, LOW);
	encoder_counter = 0;	
	while(encoder_counter < steps + 1)
	{
		delay(1);
	}
	baseStop();
	current_location += encoder_counter;
	Serial.println("OK");

}

// Stop moving base
void baseStop(void)
{
	digitalWrite(BASEA, LOW);
	digitalWrite(BASEB, LOW);

}

// move base right n steps
void baseLeft(int steps)
{
	digitalWrite(BASEA, LOW);
	digitalWrite(BASEB, HIGH);
	encoder_counter = 0;
	while(encoder_counter < steps + 1)
	{
		delay(1);
	}
	baseStop();
	current_location -= encoder_counter;
	Serial.println("OK");

}


// Lift arm all the way up, max speed
void arm_up(void)
{
	shoulder.write(ARM_UP_POSITION);
	elbow.write(ARM_UP_POSITION);
	Serial.println("OK");
}

// Lower arm all the way down, max speed
void arm_down(void)
{
	shoulder.write(ARM_DN_POSITION);
	elbow.write(ARM_DN_POSITION);
	Serial.println("OK");
}

// Lift arm to top 
void lift_arm(void)
{	 
	int pos;
	for(pos = arm_position; pos>=ARM_UP_POSITION; pos-=1)	// goes from current position to 0 degrees 
	{														 
		shoulder.write(pos);							// tell servo to go to position in variable 'pos' 
		elbow.write(pos);
		delay(15);										 // waits 15ms for the servo to reach the position 
		arm_position = pos;		
	} 
	lift_wrist(WRIST_RAISED_POSITION);
	Serial.println("OK");

}

// Lift wrist
void lift_wrist(int position)
{
	int p;
	for(p=wrist_position; p>position; p--)
	{
		wrist.write(p);
		wrist_position = p;
		delay(20);
	}
}

// Lower wrist
void lower_wrist(int position)
{
	int p;
	for(p=wrist_position; p<position; p++)
	{
		wrist.write(p);
		wrist_position = p;
		delay(50);
	}
}




