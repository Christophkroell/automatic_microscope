// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6)
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#define ANAIN_X1 33
#define ANAIN_Y1 32
#define ANAIN_X2 35
#define ANAIN_Y2 34
#define ANAIN_X3 39
#define ANAIN_Y3 36
#define ANAIN_L1 27
#define ANAIN_L2 14


class ControllerAxis {
private:
  char axis;
  int input_pin;
  int zero_point;
  int sensor_resolution = 4096;
  int dead_center = 25;
  int averaging_count = 15;
  float average, std;
  float zero_offset = 0;
  float previous_average = 0;

public:
  ControllerAxis(char axis, int input_pin) {
    this->axis = axis;
    this->input_pin = input_pin;
  }

  void update_values() {
    uint value = 0;
    uint sum = 0;
    uint sum_of_squares = 0;
    for (int i = 0; i < this->averaging_count; i++) {
      value = analogRead(this->input_pin);
      //delay(10);
      sum += value;
      sum_of_squares += value * value;
    }
    this->average = float(sum) / this->averaging_count;
    this->std = sqrt((this->averaging_count * sum_of_squares - sum * sum) / (this->averaging_count * (this->averaging_count - 1)));
    //Serial.println("average: " + String(this->average) + ", std: " + String(this->std));
  }

  void zero_sensor() {
    int orig_averaging = this->averaging_count;
    this->averaging_count = 500;
    this->update_values();
    this->averaging_count = orig_averaging;
    if (this->std == 0) {
      // error
    }
    this->zero_offset = this->average;
  }

  void send_serial_if_needed() {
    this->update_values();

    if (abs(this->average - this->previous_average) < 20) {
      return;
    }
    this->previous_average = this->average;
    int send_value = 0;
    if (abs(this->average - this->zero_offset) < this->dead_center) {
      //Serial.print(String(this->axis) + String("S") + String(send_value) + ",");
      SerialBT.print(String(this->axis) + String("S") + String(send_value) + ",");
      return;
      //return float(0);
    }
    if (this->average < this->zero_offset) {
      send_value = ((this->average / (this->zero_offset - this->dead_center)) - 1) * 1000;
      //Serial.print(String(this->axis) + String("S") + String(send_value) + ",");
      SerialBT.print(String(this->axis) + String("S") + String(send_value) + ",");
      return;
    }
    send_value = ((this->average - this->zero_offset) / (this->sensor_resolution - this->zero_offset)) * 1000;
    //Serial.print(String(this->axis) + String("S") + String(send_value) + ",");
    SerialBT.print(String(this->axis) + String("S") + String(send_value) + ",");
    return;
  }
};

// variable for storing the potentiometer value
float in_x1 = 0;
int in_y1 = 0;
int in_x2 = 0;
int in_y2 = 0;
int in_x3 = 0;
int in_y3 = 0;
ControllerAxis axis_z = ControllerAxis('Z', ANAIN_X1);
ControllerAxis axis_f = ControllerAxis('F', ANAIN_Y1);
ControllerAxis axis_x = ControllerAxis('X', ANAIN_X2);
ControllerAxis axis_y = ControllerAxis('Y', ANAIN_Y2);
ControllerAxis axis_t = ControllerAxis('T', ANAIN_X3);
ControllerAxis axis_r = ControllerAxis('R', ANAIN_Y3);
ControllerAxis axis_l1 = ControllerAxis('L', ANAIN_L1);
ControllerAxis axis_l2 = ControllerAxis('M', ANAIN_L2);

//ControllerAxis axis4 = ControllerAxis('Z', 33);


void setup() {
  Serial.begin(115200);
  SerialBT.begin("bt_controller");
  delay(1000);
  axis_z.zero_sensor();
  axis_f.zero_sensor();
  axis_x.zero_sensor();
  axis_y.zero_sensor();
  axis_t.zero_sensor();
  axis_r.zero_sensor();
}

void loop() {
  axis_z.send_serial_if_needed();
  axis_f.send_serial_if_needed();
  axis_x.send_serial_if_needed();
  axis_y.send_serial_if_needed();
  axis_t.send_serial_if_needed();
  axis_r.send_serial_if_needed();
  axis_l1.send_serial_if_needed();
  axis_l2.send_serial_if_needed();
  if (Serial.available() > 0) {
    if (Serial.read() == ',') {
      Serial.print("controller,");
    }
  }
  if (SerialBT.available() > 0) {
    if (SerialBT.read() == ',') {
      SerialBT.print("controller,");
    }
  }
}