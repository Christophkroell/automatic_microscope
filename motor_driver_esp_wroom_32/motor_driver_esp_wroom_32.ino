#include <AccelStepper.h>

#define HALFSTEP 8

//todo if xyr or ...
#define device_name "motor_controller_XYR,"
#define motor1_str 'X'
#define motor1_pin1  32    // IN1 on ULN2003 ==> Blue   on 28BYJ-48
#define motor1_pin2  33    // IN2 on ULN2004 ==> Pink   on 28BYJ-48
#define motor1_pin3  25    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
#define motor1_pin4  26    // IN4 on ULN2003 ==> Orange on 28BYJ-48
//#define motor1_pin1  23  // IN1 on ULN2003 ==> Blue   on 28BYJ-48 
//#define motor1_pin2  22  // IN2 on ULN2004 ==> Pink   on 28BYJ-48
//#define motor1_pin3  1   // IN3 on ULN2003 ==> Yellow on 28BYJ-48
//#define motor1_pin4  3   // IN4 on ULN2003 ==> Orange on 28BYJ-48
#define home_sensor1_pin 13

#define motor2_str 'Y'
#define motor2_pin1  18    // IN1 on ULN2003 ==> Blue   on 28BYJ-48
#define motor2_pin2  5    // IN2 on ULN2004 ==> Pink   on 28BYJ-48
#define motor2_pin3  17     // IN3 on ULN2003 ==> Yellow on 28BYJ-48
#define motor2_pin4  16    // IN4 on ULN2003 ==> Orange on 28BYJ-48
#define home_sensor2_pin 13

#define motor3_str 'R'
#define motor3_pin1  4    // IN1 on ULN2003 ==> Blue   on 28BYJ-48
#define motor3_pin2  2     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
#define motor3_pin3  15     // IN3 on ULN2003 ==> Yellow on 28BYJ-48 // works but high
#define motor3_pin4  19    // IN4 on ULN2003 ==> Orange on 28BYJ-48
#define home_sensor3_pin 13


#define led1_pin 22
#define led2_pin 22

unsigned long start_millis;
unsigned long current_millis;
uint loops_per_second = 0;
const unsigned long log_period = 25;
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
    //float mm_per_step;
    //int steps_per_rotation;
    //float limit_min_mm;
    //float limit_max_mm;
    int home_pin;
    int home_speed;
    //float home_speed_mms;
    //int home_direction;
    //float speed_mms;
    int speed;
    //float speed_mode_speed_mms;
    int speed_mode_speed;
    int max_speed;
    int acceleration;
    //float max_speed_mms;
    //float acceleration_mms2;
    

  public:
    MOTOR_STATE state;

    Motor(AccelStepper stepper, int home_pin) {
      this->stepper = stepper;
      this->home_pin = home_pin;
      pinMode(this->home_pin, INPUT_PULLUP);
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
      this->stepper.setMaxSpeed(this->max_speed);
      this->stepper.setAcceleration(this->acceleration);
    }
    
    void set_default_parameter() {
      //this->speed_mms = 5;
      //this->speed_mode_speed_mms = 0;
      this->state = MOTOR_STATE::UNKNOWN;
      //this->steps_per_rotation = 4096;
      //float circumverence_mm = 35.36;
      //this->mm_per_step = circumverence_mm / this->steps_per_rotation;
      //this->home_speed_mms = 1;
      //this->home_direction = 1;
      //this->limit_min_mm = -0.01;
      //this->limit_max_mm = 100;
      //this->max_speed_mms = 12.5;
      //this->acceleration_mms2 = 4;
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
          //Serial.println("current position: " + String(this->stepper.currentPosition()) + ", speed: " + String(this->stepper.speed()));
        }
      }
      if (this->state == MOTOR_STATE::UNKNOWN) {
        return;
      }
      if (this->state == MOTOR_STATE::HOMING) {
        if (this->is_home_sensor_active()) {
          this->stepper.setCurrentPosition(0);
          this->state = MOTOR_STATE::READY;
          Serial.print("home,");
          return;
        }
        if (!this->stepper.isRunning()) {
          this->state = MOTOR_STATE::ERROR;
          Serial.print("not_home,");
          return;
        }
      }
      if (this->state == MOTOR_STATE::SPEEDMODE) {
        this->stepper.setSpeed(this->speed_mode_speed);
        this->stepper.runSpeed();
        //Serial.println("in step mode");
      }
    }

    void set_this_zero() {
      this->stepper.setCurrentPosition(0);
    }

    /*
    float mm_to_steps(float mms) {
      float steps = mms / this->mm_per_step;
      //Serial.println("mms: " + String(mms) + " to steps: " + String(steps));
      return steps;
    }
    
    void setSpeed_mms(float speed_mms) {
      this->stepper.setMaxSpeed(this->mm_to_steps(speed_mms));
    }*/

    void go_to_step(long new_pos) {
      if (this->state != MOTOR_STATE::READY){
        Serial.println("can not do absolute move. Motor not in READY state");
        return;
      }
      this->stepper.moveTo(new_pos);
    }

    void go_steps(long steps_to_move) {
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
      this->stepper.move(steps_to_move);
    }
    
    bool set_state_to_speedmode() {
      if (this->state == MOTOR_STATE::READY || this->state == MOTOR_STATE::SPEEDMODE || this->state == MOTOR_STATE::UNKNOWN) {
        this->state = MOTOR_STATE::SPEEDMODE;
        //Serial.println("was set to speed mode");
      }
    }

    void set_speed_mode_speed(float speed) {
      this->speed_mode_speed = speed;
    }

    void do_home() {
      //Serial.print("Home sequence started, with speed: ");
      //Serial.println(this->home_speed);
      //this->stepper.
      //this->setSpeed_mms(this->home_speed_mms);
      //long max_movment_distance = (this->limit_max_mm - this->limit_min_mm) * this->home_direction;
      //Serial.print("Will move to: ");
      //Serial.println(max_movment_distance);
      //this->move_mm(max_movment_distance);
      //this->state = MOTOR_STATE::HOMING;
    }

};

class Led {
  private:
    int pwm_channel;

  public:
    Led(int pin, int pwm_channel) {
      this->pwm_channel = pwm_channel;
      ledcSetup(pwm_channel, 1000, 8);
      ledcAttachPin(pin, pwm_channel);
      ledcWrite(pwm_channel, 0);
    }

    void set_to(int(brightness)){
      ledcWrite(this->pwm_channel, brightness);
    }
};

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
Motor motor_a(stepper1, home_sensor1_pin);
Motor motor_b(stepper2, home_sensor2_pin);
Motor motor_c(stepper3, home_sensor3_pin);
Led led_a(led1_pin, 0);
Led led_b(led2_pin, 1);

String received_string;
int current_direction = 1;

// message shape
// axis: X, Y, C
// mode: H (home), A (absolute move mm), R (relative move mm), 


void parse_string() {
  Motor* used_motor;
  Led* used_led;
  bool is_led = false;
  if (received_string.length() == 0) {
    Serial.print(device_name);
    return;    
  }
  char axis = received_string.charAt(0);
  char mode = received_string.charAt(1);
  switch(axis) {
    case motor1_str:
      used_motor = &motor_a;
      break;
    case motor2_str:
      used_motor = &motor_b;
      break;
    case motor3_str:
      used_motor = &motor_c;
      break;
    case 'L':
      used_led = &led_a;
      is_led = true;
      break;
    case 'M':
      used_led = &led_b;
      is_led = true;
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
  if (is_led) {
    used_led->set_to(int(distance_or_speed));
    return;
  }
  switch(mode) {
    case 'H':
      used_motor->do_home();
      return;
    case 'A':
      used_motor->go_to_step(long(distance_or_speed));
      break;
    case 'R':
      used_motor->go_steps(long(distance_or_speed));
      break;
    case 'S':
      used_motor->set_speed_mode_speed(distance_or_speed);
      used_motor->set_state_to_speedmode();
      break;
    case 'E':
      used_motor->reset();
      break;
  }
  return;
}


void setup()
{
  Serial.begin(115200);
  motor_a.set_defaults();
  motor_b.set_defaults();
  motor_c.set_defaults();
  start_millis = millis();
  received_string = "";
}

void handle_serial_data() {
  char data = Serial.read();
  if (data == '\n') {
    return;
  }
  if (data != ',') {
    received_string += data;
    //Serial.println("received_string: " + String(received_string));
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
  //  do_log = true;
    //Serial.println("loops_per_second: " + String(loops_per_second));
  //  loops_per_second = 0;
  }

  if (Serial.available() > 0) {
    handle_serial_data();
  }
  motor_a.run();
  motor_b.run();
  motor_c.run();
}
