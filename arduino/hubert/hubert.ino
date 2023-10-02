#define N_SERVOS 5 // We are not using the gripper

#include <Arduino.h>
#include <Servo.h>

//Servos
Servo servos[N_SERVOS];
const int servo_pins[N_SERVOS] = {3, 9, 10, 5, 6};

Servo launcher;
const int launcher_pin = 11;

// Servo position
int init_pos[N_SERVOS] = {1600, 2200, 1410, 1500, 2100};
int curr_pos[N_SERVOS];
int move_pos[N_SERVOS];
float servo_vel[N_SERVOS];
float curr_pos_float[N_SERVOS];

// Launcher position
const int launcher_min = 600;
const int launcher_max = 2300;
int launcher_pos = launcher_min;
int launcher_target = launcher_min;
int launcher_vel;
const int launcher_steps_per_epoch = 12;

// Servo speed
const int steps_per_epoch = 6;

// Servo limits
const int pos_min[N_SERVOS] = {560, 750, 550, 550, 950};
const int pos_max[N_SERVOS] = {2330, 2300, 2400, 2340, 2400};

// Timings
const long interval = 20;           // Servos operate on 50Hz -> delay of 20 ms
unsigned long previousMillis = 0;   // Will store the previous time the servos were updated

// Flags
bool targetUpdate = false;
bool launching = false;

// Status flags
const unsigned char LAUNCH_FLAG = 1;
const unsigned char MOVE_FLAG = 1 << 1;


bool time_to_update_servo() {
  unsigned long currentMillis = millis();
  if ( currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

//Servo update function
void update_servo_pos() {
  /* 
  Update the location of all servos in Hubert
  */

  // Check if it is time to update
  if ( !time_to_update_servo() ) return;

  update_pose();
  update_launcher();
}

void update_launcher() {
  /* 
  Update the servo in the launcher
  */
  if (!launching) return;

  if ( abs(launcher_target - launcher_pos) <= launcher_steps_per_epoch ) {
    // Check if the the location is close the the target
    launcher_pos = launcher_target;

    // Go back
    if ( launcher_target == launcher_max ) {
      launcher_target = launcher_min;
      launcher_vel = -launcher_steps_per_epoch;
    }
    // Reset the launch seqence
    else launching = false;
  }
  else {
    // Move closer to the target
    launcher_pos += launcher_vel;
  }

  // Keep pos within bounds
  if ( launcher_pos < launcher_min) launcher_pos = launcher_min;
  else if ( launcher_pos > launcher_max) launcher_pos = launcher_max;

  launcher.write(launcher_pos);
}

void start_launch_sequence() {
  if ( launching ) return; // Cant start in the middle of a launch

  launching = true;
  launcher_target = launcher_max;
  launcher_vel = launcher_steps_per_epoch;
}

void update_pose() {
  /* 
  Update the servos resposible for Huberts pose
  */
  for ( byte i = 0; i < N_SERVOS; i++ ) {
    
    if ( abs(move_pos[i] - curr_pos[i]) <= steps_per_epoch ) {
      // Check if the the location is close the the target
      curr_pos[i] = move_pos[i];
    }
    else {
      // Move closer to the target
      curr_pos_float[i] += servo_vel[i];
      curr_pos[i] = round(curr_pos_float[i]);
    }

    // Keep pos within bounds
    if ( curr_pos[i] < pos_min[i]) curr_pos[i] = pos_min[i];
    else if ( curr_pos[i] > pos_max[i]) curr_pos[i] = pos_max[i];

    servos[i].writeMicroseconds(curr_pos[i]);
  }
}

void update_target_pose() {
  /*
  Update the target position for every servo in Huberts body
  */
  int steps_to_pos[N_SERVOS];

  if ( !targetUpdate ) return;
  targetUpdate = false;

  // Calculate the distance to the new position and reset the current steps
  // Also save the maximum distance
  int max_dist = 0;
  for ( byte i = 0; i < N_SERVOS; i++ ) {
    int dist = move_pos[i] - curr_pos[i];
    steps_to_pos[i] = dist;
    curr_pos_float[i] = float(curr_pos[i]);
    if ( abs(dist) > max_dist ) {
      max_dist = abs(dist);
    }
  }

  if ( max_dist == 0 ) {
    // No new movement
    for ( byte i = 0; i < N_SERVOS; i++ ) servo_vel[i] = 0.0;
  }

  // Calculate the time to complete the movment
  float time = float(max_dist) / steps_per_epoch;

  // Set the velocity for each joint
  for ( byte i = 0; i < N_SERVOS; i++ ) {
    servo_vel[i] = steps_to_pos[i] / time;
  }
}

void read_target_pose() {
  const int buffer_size = 2 * N_SERVOS;
  byte buffer[buffer_size];

  while ( Serial.available() < buffer_size ) continue;

  // Read 10 bytes into the buffer
  for (int i = 0; i < buffer_size; i++) {
    buffer[i] = Serial.read();
  }

  // Convert the 10 bytes into 5 integers in big endian format
  for (int i = 0; i < N_SERVOS; i++) {
    move_pos[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
  }

  targetUpdate = true;
}

void write_curr_pose() {
  for (int i = 0; i < N_SERVOS; i++) {
    byte highByte = curr_pos[i] >> 8;
    byte lowByte = curr_pos[i] & 0xFF;
    Serial.write(highByte);
    Serial.write(lowByte);
  }
}

void send_status() {
  unsigned char status_flag = 0;

  if ( launching ) status_flag |= LAUNCH_FLAG;

  bool moving = false;
  for (int i = 0; i < N_SERVOS; i++) {
    if ( move_pos[i] != curr_pos[i] ) moving = true;
  }
  if ( moving ) status_flag |= MOVE_FLAG;

  Serial.write(status_flag);
}

void readSerial() {
  if (Serial.available() > 0) {
    char command = Serial.read(); // Read the next character from the buffer

    switch (command) {
      case 'm':
        read_target_pose();        // Read a new position to move to
        break;
      case 'g':
        write_curr_pose();
        break;
      case 's':
        send_status();
        break;
      case 'l':
        start_launch_sequence();
        break;
      // Add more cases for other characters if needed
      default:
        // Handle unknown commands
        break;
    }
  }
}

void setup() {

  Serial.begin(57600); // Starts the serial communication

	//Attach each joint servo
  for ( byte i = 0; i < N_SERVOS; i++ ) {
    servos[i].attach(servo_pins[i]);
    curr_pos[i] = init_pos[i];
    move_pos[i] = init_pos[i];
    servos[i].writeMicroseconds(curr_pos[i]);
  }

  // Attach the launcher
  launcher.attach(launcher_pin);
  launcher.writeMicroseconds(launcher_pos);
}

void loop() {
  readSerial();
  update_target_pose();
  update_servo_pos();
}
