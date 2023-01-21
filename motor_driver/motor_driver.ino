#include <AccelStepper.h>
unsigned long start_millis;
unsigned long current_millis;
uint loops_per_second = 0;
const unsigned long log_period = 1000;
bool do_log;

enum class MOTOR_STATE{
  UNKNOWN,
  HOMING,
  READY,
  MOVING,
  ERROR,
  SPEEDMODE
  };

class Motor {
  private:
    AccelStepper stepper;
    float mm_per_step;
    int steps_per_rotation;
    float limit_min_mm;
    float limit_max_mm;
    int home_pin;
    float home_speed_mms;
    int home_direction;
    float speed_mms;
    float speed_mode_speed_mms;
    float max_speed_mms;
    float acceleration_mms2;
    

  public:
    MOTOR_STATE state;

    Motor(AccelStepper stepper, int home_pin) {
      Serial.println("WHAT EVER");
      this->stepper = stepper;
      this->home_pin = home_pin;
    }

    void reset() {
      this->state = MOTOR_STATE::UNKNOWN;
      this->stepper.stop();
    }

    void set_defaults(){
      this->set_default_parameter();
      this->update_stepper();
    }
    void update_stepper() {
      this->stepper.setMaxSpeed(this->mm_to_steps(this->max_speed_mms));
      this->stepper.setAcceleration(this->mm_to_steps(this->acceleration_mms2));
      pinMode(this->home_pin, INPUT_PULLUP);
    }
    
    void set_default_parameter() {
      this->speed_mms = 5;
      this->speed_mode_speed_mms = 0;
      this->state = MOTOR_STATE::UNKNOWN;
      this->steps_per_rotation = 4096;
      float circumverence_mm = 35.36;
      this->mm_per_step = circumverence_mm / this->steps_per_rotation;
      Serial.println("mm per step: " + String(this->mm_per_step));
      this->home_speed_mms = 1;
      this->home_direction = 1;
      this->limit_min_mm = -0.01;
      this->limit_max_mm = 100;
      this->max_speed_mms = 12.5;
      this->acceleration_mms2 = 4;
    }
    
    bool is_home_sensor_active() {
      if (digitalRead(this->home_pin) == 0) {
        return true;
      }
      return false;
    }

    void run() {
      
      this->stepper.run();
      if (do_log){
        if (this->stepper.isRunning()){
          Serial.println("current position: " + String(this->stepper.currentPosition()) + ", speed: " + String(this->stepper.speed()));
        }
      }
      if (this->state == MOTOR_STATE::UNKNOWN) {
        return;
      }
      if (this->state == MOTOR_STATE::HOMING) {
        if (this->is_home_sensor_active()) {
          this->stepper.setCurrentPosition(0);
          this->state = MOTOR_STATE::READY;
          Serial.println("Home sensor found");
          return;
        }
        if (!this->stepper.isRunning()) {
          this->state = MOTOR_STATE::ERROR;
          Serial.println("Home not found");
          return;
        }
      }
      if (this->state == MOTOR_STATE::SPEEDMODE) {
        this->stepper.setSpeed(this->mm_to_steps(speed_mode_speed_mms));
        this->stepper.runSpeed();
        //Serial.println("in step mode");
      }
    }

    void set_this_zero() {
      this->stepper.setCurrentPosition(0);
    }

    float mm_to_steps(float mms) {
      float steps = mms / this->mm_per_step;
      //Serial.println("mms: " + String(mms) + " to steps: " + String(steps));
      return steps;
    }
    
    void setSpeed_mms(float speed_mms) {
      this->stepper.setMaxSpeed(this->mm_to_steps(speed_mms));
    }

    void go_pos_mm(long new_pos) {
      if (this->state != MOTOR_STATE::READY){
        Serial.println("can not do absolute move. Motor not in READY state");
        return;
      }
      this->stepper.moveTo(this->mm_to_steps(new_pos));
    }

    void move_mm(long mm_to_move) {
      if (this->state == MOTOR_STATE::ERROR) {
        Serial.println("Move not allowed, Motor in ERROR state");
        return;
      }
      if (this->state == MOTOR_STATE::HOMING) {
        Serial.println("Move not allowed, Motor is HOMING");
        return;
      }
      if (this->state == MOTOR_STATE::MOVING) {
        Serial.println("Move not allowed, Motor is MOVING");
        return;
      }
      long steps_to_move = mm_to_steps(mm_to_move);
      Serial.print("go to step:");
      Serial.println(steps_to_move);
      this->stepper.move(steps_to_move);
    }
    
    bool set_state_to_speedmode() {
      if (this->state == MOTOR_STATE::READY || this->state == MOTOR_STATE::SPEEDMODE || this->state == MOTOR_STATE::UNKNOWN) {
        this->state = MOTOR_STATE::SPEEDMODE;
        Serial.println("was set to speed mode");
      }
    }

    void set_speed_mode_speed_mms(float speed_mms) {
      if (speed_mms > this->max_speed_mms) {
        speed_mms = this->max_speed_mms;
      }
      this->speed_mode_speed_mms = speed_mms;
    }

    void do_home() {
      Serial.print("Home sequence started, with speed: ");
      Serial.println(this->home_speed_mms);
      this->setSpeed_mms(this->home_speed_mms);
      long max_movment_distance = (this->limit_max_mm - this->limit_min_mm) * this->home_direction;
      Serial.print("Will move to: ");
      Serial.println(max_movment_distance);
      this->move_mm(max_movment_distance);
      this->state = MOTOR_STATE::HOMING;
    }

};

#define HALFSTEP 8

//#define motorPin1  32//8     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
//#define motorPin2  33//9     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
//#define motorPin3  25//10    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
//#define motorPin4  26//11    // IN4 on ULN2003 ==> Orange on 28BYJ-48

//#define motorPin1  19     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
//#define motorPin2  21     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
//#define motorPin3  22   // IN3 on ULN2003 ==> Yellow on 28BYJ-48
//#define motorPin4  23    // IN4 on ULN2003 ==> Orange on 28BYJ-48


//#define motorPin4  13    // IN4 on ULN2003 ==> Orange on 28BYJ-48
//#define motorPin1  12     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
//#define motorPin2  14     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
//#define motorPin3  27   // IN3 on ULN2003 ==> Yellow on 28BYJ-48

#define motor1_pin1  19     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
#define motor1_pin2  21     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
#define motor1_pin3  22   // IN3 on ULN2003 ==> Yellow on 28BYJ-48
#define motor1_pin4  23    // IN4 on ULN2003 ==> Orange on 28BYJ-48
#define home_sensor1_pin 12

#define motor2_pin1  16     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
#define motor2_pin2  17     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
#define motor2_pin3  5    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
#define motor2_pin4  18    // IN4 on ULN2003 ==> Orange on 28BYJ-48
#define home_sensor2_pin 13

#define motor3_pin1  15     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
#define motor3_pin2  2     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
#define motor3_pin3  0    // IN3 on ULN2003 ==> Yellow on 28BYJ-48 // works but high
#define motor3_pin4  4    // IN4 on ULN2003 ==> Orange on 28BYJ-48
#define home_sensor3_pin 14

// not working pins 0,1,9,10,11,34,35,36,39

int endPoint = 1024;        // Move this many steps - 1024 = approx 1/4 turn
int home_sensor_state_m1 = 0;
int home_sensor_state_m2 = 0;
int home_sensor_state_m3 = 0;
int new_speed_m1 = 25;
int new_speed_m2 = 25;
int new_speed_m3 = 25;

// NOTE: The sequence 1-3-2-4 is required for proper sequencing of 28BYJ-48
AccelStepper stepper1(HALFSTEP, motor1_pin1, motor1_pin3, motor1_pin2, motor1_pin4);
AccelStepper stepper2(HALFSTEP, motor2_pin1, motor2_pin3, motor2_pin2, motor2_pin4);
AccelStepper stepper3(HALFSTEP, motor3_pin1, motor3_pin3, motor3_pin2, motor3_pin4);
Motor motor_x(stepper1, home_sensor1_pin);
Motor motor_y(stepper2, home_sensor2_pin);
Motor motor_c(stepper3, home_sensor3_pin);



String received_string;
int current_direction = 1;

// message shape
// axis: X, Y, C
// mode: H (home), A (absolute move mm), R (relative move mm), 


void parse_string() {
  Motor* used_motor;
  char axis = received_string.charAt(0);
  char mode = received_string.charAt(1);
  switch(axis) {
    case 'X':
      used_motor = &motor_x;
      break;
    case 'Y':
      used_motor = &motor_y;
      break;
    case 'C':
      used_motor = &motor_c;
      break;
  }
  float distance_or_speed = 0;
  if (received_string.length() > 2) {
    distance_or_speed = received_string.substring(2).toFloat();
  }
  Serial.println("parse string: " + received_string + 
    ", Axis: " + String(axis) + 
    ", Mode: " + String(mode) +
    ", Speed or Distance: " + String(distance_or_speed));
  switch(mode) {
    case 'H':
      used_motor->do_home();
      return;
    case 'A':
      used_motor->go_pos_mm(distance_or_speed);
      break;
    case 'R':
      used_motor->move_mm(distance_or_speed);
      break;
    case 'S':
      used_motor->set_speed_mode_speed_mms(distance_or_speed);
      used_motor->set_state_to_speedmode();
      break;
    case 'E':
      used_motor->reset();
      break;
  }
  return;
  //Serial.println("used_motor: " + String(used_motor));
  if (received_string.indexOf('X') > 0) {
    
    Serial.println("go to mm: " + String(120*current_direction));
    motor_x.set_this_zero();
    motor_x.go_pos_mm(100*current_direction);
    current_direction *= -1;
  }
  if (received_string.indexOf('Y') > 0) {
    motor_y.do_home();
  }
  if (received_string.indexOf('C') > 0) {
    motor_c.do_home();
  }
}

void setup()
{
  Serial.begin(115200);
  motor_x.set_defaults();
  motor_y.set_defaults();
  motor_c.set_defaults();
  start_millis = millis();
  received_string = "";
  String tests = "XS0.1";
  Serial.println(tests);
  Serial.println(tests.substring(2));
  Serial.println(tests.substring(2).toFloat());
}

void handle_serial_data() {
  char data = Serial.read();
  if (data == '\n') {
    return;
  }
  if (data != ',') {
    received_string += data;
    Serial.println("received_string: " + String(received_string));
    return;
  }
  parse_string();
  received_string = "";
}

void loop()
{
  loops_per_second += 1;
  do_log = false;
  current_millis = millis();
  if (current_millis - start_millis >= log_period) {
    start_millis = current_millis;
    do_log = true;
    //Serial.println("loops_per_second: " + String(loops_per_second));
    loops_per_second = 0;
  }

  if (Serial.available() > 0) {
    // see message shape
    // Axis, single axis. for example X
    // Mode,
    handle_serial_data(); 
    //char data = Serial.read();
    //if (data == '\n') {
    //  break;
    //}
    //if (data != ',') {
    //  received_string += data;
      
    //} else {
    //  parse_string();
    //  received_string = "";
    //}
  }

  motor_x.run();
  motor_y.run();
  motor_c.run();
  //stepper3.setSpeed(new_speed_m3);
  //stepper3.runSpeed();
  //Change direction at the limits
  //  if (stepper1.distanceToGo() == 0)
  // {
  //   Serial.println(stepper1.currentPosition());
  //   stepper1.setCurrentPosition(0);
  //   endPoint = -endPoint;
  //   stepper1.moveTo(endPoint);
  //   Serial.println(stepper1.currentPosition());
  // }
  //  stepper1.run();
}