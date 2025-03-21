/*------------KRS--------------------
#ESP-Door-Automation
#Threshlod Value tested is 1200
#UART--ESP to Raspberry Pi 4 
#-------LED_STATUS_PINS--------------
#Blue->IR detection pin
#Red->Esp enable Pin
#Green->Face Recognistion Sucesssful
#Yellow->Face Recognisition not,
#sucessful no match found in database
#Pins will controlled locally or controlled by Raspberry by UART
#-------------------------------------
#----------IR_PIN_CHECK---------------
#Sends continous data of IR values,
#in String using UART,CONTINOUSLY
#-------------------------------------
#--------------LIDAR----------------
#Lidar will send data(LEFT,RIGHT) or (RIGHT,LEFT) through UART
#The algothirm will run locally here
#------------------------------------
*/

/*---CASES----
        # Total number of cases are 6
        # LIDAR="GetLIDAR";
        # reset="RESET";
        # RECOG_1="RECOGON";
        # RECOG_2="RECOGOFF";
        # RECOG_ERROR_1="NORECOGON";
        # RECOG_ERROR_2="NORECOGOFF"; 
        */


#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Define which Wire objects to use, may depend on platform
// or on your configurations.
#define SENSOR1_WIRE Wire
#define SENSOR2_WIRE Wire

//pin declaration
//#define IR_PIN A0 // IR Digital Pin
#define LED_IR 13 //OFF->IR hasn't detected, ON->IR Detected  //yellow
#define LED_RECOG 12//ON->when person is recognised //green
#define LED_NOTRECOG 14//ON->when person is not recognised and is not in database //RED
#define ESP_enabled 27//ON->when esp is running //blue

//String declaration for cases
const String LIDAR="GetLIDAR";
const String reset="RESET";
const String RECOG_1="RECOGON";
const String RECOG_2="RECOGOFF";
const String RECOG_ERROR_1="NORECOGON";
const String RECOG_ERROR_2="NORECOGOFF";
const String IR_DETECT="GetIR";

//values
String data;
char Right='R';
char Left='L';
char Error='E';
const int average_val=3;


//thresholds
#define IR_Threshold 150
#define LIDAR_Threshold 1200

//flags
volatile bool IR_FLAG=false;
volatile bool RECOG_FLAG=false;
volatile bool NORECOG_FLAG=false;
const int time_const=5000;


// Setup mode for doing reads
typedef enum {
  RUN_MODE_DEFAULT = 1,
  RUN_MODE_CONT
} runmode_t;

runmode_t run_mode = RUN_MODE_DEFAULT;
uint8_t show_command_list = 1;

typedef struct {
  Adafruit_VL53L0X *psensor; // pointer to object
  TwoWire *pwire;
  int id;            // address
  int shutdown_pin;  // shutdwon pin
  Adafruit_VL53L0X::VL53L0X_Sense_config_t
      sensor_config;     // options for how to use the sensor
  uint16_t range;        // range value used in continuous mode stuff.
  uint8_t sensor_status; // status from last ranging in continuous.
} sensorList_t;

// object
Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;
Adafruit_VL53L0X sensor3;

// Setup for 2 sensors
sensorList_t sensors[] = {
#ifndef ARDUINO_ARCH_AVR //
     {&sensor1, &SENSOR1_WIRE, 0x32, 5,
     Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE, 0, 0},
    {&sensor2, &SENSOR2_WIRE, 0x31, 18,
     Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE, 0, 0},
     {&sensor3, &SENSOR2_WIRE, 0x33, 19,
     Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE, 0, 0},
#endif
};

const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

const uint16_t ALL_SENSORS_PENDING = ((1 << COUNT_SENSORS) - 1);
uint16_t sensors_pending = ALL_SENSORS_PENDING;
uint32_t sensor_last_cycle_time;

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then
   set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but
   0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its
   XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but
   0x29 and whatever you set the first sensor to
*/

void Initialize_sensors() {
  bool found_any_sensors = false;
  // Set all shutdown pins low to shutdown sensors
  for (int i = 0; i < COUNT_SENSORS; i++)
    digitalWrite(sensors[i].shutdown_pin, LOW);
  delay(10);

  for (int i = 0; i < COUNT_SENSORS; i++) {
    // one by one enable sensors and set their ID
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(10); // give time to wake up.
    if (sensors[i].psensor->begin(sensors[i].id, false, sensors[i].pwire,
                                  sensors[i].sensor_config)) {
      found_any_sensors = true;
    } else {
      Serial.print(i, DEC);
      Serial.print(F(": failed to start\n"));
    }
  }
  if (!found_any_sensors) {
    Serial.println("No valid sensors found");
    while (1)
      ;
  }
}

void status_initialize(){
  pinMode(LED_IR,OUTPUT);
  pinMode(LED_RECOG,OUTPUT);
  pinMode(LED_NOTRECOG,OUTPUT);
  pinMode(ESP_enabled,OUTPUT);
}
//====================================================================
// LIDAR
//====================================================================
void read_sensors() {
  // First use simple function
  uint16_t ranges_mm[COUNT_SENSORS];
  bool timeouts[COUNT_SENSORS];
  uint32_t stop_times[COUNT_SENSORS];

  digitalWrite(13, HIGH);
  uint32_t start_time = millis();
  for (int i = 0; i < COUNT_SENSORS; i++) {
    ranges_mm[i] = sensors[i].psensor->readRange();
    timeouts[i] = sensors[i].psensor->timeoutOccurred();
    stop_times[i] = millis();
  }
  uint32_t delta_time = millis() - start_time;
  digitalWrite(13, LOW);

  Serial.print(delta_time, DEC);
  Serial.print(F(" "));
  for (int i = 0; i < COUNT_SENSORS; i++) {
    Serial.print(i, DEC);
    Serial.print(F(":"));
    Serial.print(ranges_mm[i], DEC);
    Serial.print(F(" "));
    Serial.print(stop_times[i] - start_time, DEC);
    if (timeouts[i])
      Serial.print(F("(TIMEOUT) "));
    else
      Serial.print(F("          "));
    start_time = stop_times[i];
  }
  Serial.println();
}
//===============================================================
// LIDAR_Continuous range test code
//===============================================================

void start_continuous_range(uint16_t cycle_time) {
  if (cycle_time == 0)
    cycle_time = 100;
  Serial.print(F("start Continuous range mode cycle time: "));
  Serial.println(cycle_time, DEC);
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    sensors[i].psensor->startRangeContinuous(cycle_time); // do 100ms cycle
  }
  sensors_pending = ALL_SENSORS_PENDING;
  sensor_last_cycle_time = millis();
}

void stop_continuous_range() {
  Serial.println(F("Stop Continuous range mode"));
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    sensors[i].psensor->stopRangeContinuous();
  }
  delay(100); // give time for it to complete.
}

void Process_continuous_range() {

  uint16_t mask = 1;
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    bool range_complete = false;
    if (sensors_pending & mask) {
      if (range_complete) {
        sensors[i].range = sensors[i].psensor->readRangeResult();
        sensors[i].sensor_status = sensors[i].psensor->readRangeStatus();
        sensors_pending ^= mask;
      }
    }
    mask <<= 1; // setup to test next one
  }
  // See if we have all of our sensors read OK
  uint32_t delta_time = millis() - sensor_last_cycle_time;
  if (!sensors_pending || (delta_time > 1000)) {
    digitalWrite(13, !digitalRead(13));
    Serial.print(delta_time, DEC);
    Serial.print(F("("));
    Serial.print(sensors_pending, HEX);
    Serial.print(F(")"));
    mask = 1;
    for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
      Serial.print(F(" : "));
      if (sensors_pending & mask)
        Serial.print(F("TTT")); // show timeout in this one
      else {
        Serial.print(sensors[i].range, DEC);
        if (sensors[i].sensor_status == VL53L0X_ERROR_NONE)
          Serial.print(F("  "));
        else {
          Serial.print(F("#"));
          Serial.print(sensors[i].sensor_status, DEC);
        }
      }
    }
    // setup for next pass
    Serial.println();
    sensor_last_cycle_time = millis();
    sensors_pending = ALL_SENSORS_PENDING;
  }
}

//====================================================================
// LIDAR_SENDING_DATA
//====================================================================
void Lidar() {
    bool sensor_1 = false;
    bool sensor_2 = false;
    unsigned long start_time_1 = millis();

    while (millis() - start_time_1 < 3000) {
        int distance_sensor1 = sensors[0].psensor->readRange();
        int distance_sensor2 = sensors[1].psensor->readRange();

        if (distance_sensor1 < 0 || distance_sensor2 < 0) {
            Serial.println("Invalid distance reading");
            Serial.flush();
            return; 
        }

        if (sensors[0].psensor->timeoutOccurred() || sensors[1].psensor->timeoutOccurred()) {
            Serial.println("TIMEOUT");
            Serial.flush();
            return; 
        }

        if (distance_sensor1 < LIDAR_Threshold && distance_sensor2 > LIDAR_Threshold) {
            sensor_1 = true;
            sensor_2 = false;
            break; 
        } else if (distance_sensor2 < LIDAR_Threshold && distance_sensor1 > LIDAR_Threshold) {
            sensor_1 = false;
            sensor_2 = true;
            break;
        }

        yield(); 
    }

    unsigned long start_time_2 = millis();
    while (millis() - start_time_2 < 3000) {
        int distance_sensor1 = sensors[0].psensor->readRange();
        int distance_sensor2 = sensors[1].psensor->readRange();

        if (distance_sensor1 < 0 || distance_sensor2 < 0) {
            Serial.println("Invalid distance reading");
            return; 
        }

        if (sensors[0].psensor->timeoutOccurred() || sensors[1].psensor->timeoutOccurred()) {
            Serial.println("TIMEOUT");
            Serial.flush();
            return; 
        }

        if (!sensor_2 && distance_sensor2 < LIDAR_Threshold) {
            Serial.println("Entry");
            Serial.flush();
            break;
        } 
      
        else if (!sensor_1 && distance_sensor1 < LIDAR_Threshold) {
            Serial.println("Exit");
            Serial.flush();
            break;
        }
        yield();
    }
    if (!sensor_1 && !sensor_2) {
        Serial.println("No one passed");
        Serial.flush();
    }
}

//====================================================================
// IR
//====================================================================
void Serial_IR_Send(){
  if(IR_FLAG==true){
    Serial.println("DETECTED");
    Serial.flush();
    //Serial.write("\n");
  }
  else{
    Serial.println("NOT DETECTED");
    Serial.flush();
    //Serial.write("\n");
  }
}
//status set function
void Status_Set(){
  if(IR_FLAG==true){
    digitalWrite(LED_IR,HIGH);
  }
  else{
    digitalWrite(LED_IR,LOW);
  }
}
//IR main logic function
void IR_Reading_Continuous() {
    int distance=0;
    int distance_sensor3 = sensors[2].psensor->readRange();
    for(int i=0; i<=5 ; i++){
      distance=distance+distance_sensor3;
    }
    distance=distance/5;
    if(distance <= 600){
      Serial.println("detected");
      Serial.flush();
      digitalWrite(LED_IR, HIGH);
    }
    else{
      Serial.println("not detected");
      Serial.flush();
      digitalWrite(LED_IR, LOW);
    }
    //Serial.println(distance_sensor3);
 
    }


  TaskHandle_t Task1;
  TaskHandle_t Task2;
  
  void task1(void *pvParameters) {
    for (;;) {
          IR_Reading_Continuous();
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

  void task2(void *pvParameters) {
    for (;;) {
         if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim(); 
        
        if (command == "LIDAR") {
            Lidar();
        } else if (command == "IR_DETECT") {
            IR_Reading_Continuous();
        } else if (command == reset) {
            ESP.restart();
        } else if (command == "RECOG_1") {
            digitalWrite(LED_RECOG, HIGH);
            RECOG_FLAG = true;
        } else if (command == "RECOG_2") {
            digitalWrite(LED_RECOG, LOW);
            RECOG_FLAG = false;
        } else if (command == "RECOG_ERROR_1") {
            digitalWrite(LED_NOTRECOG, HIGH);
            NORECOG_FLAG = true;
        } else if (command == "RECOG_ERROR_2") {
            digitalWrite(LED_NOTRECOG, LOW);
            NORECOG_FLAG = false;
        } else {
            // Default case
            Serial.write("error"); // Error or unknown command
            Serial.flush();
        }
    }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }


//===================================================================
// Setup
//====================================================================

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // wait until serial port opens ... For 5 seconds max
  while (!Serial && millis() < 5000)
    ;
  status_initialize();
  // initialize all of the pins.
  //Serial.println(F("VL53LOX_multi start, initialize IO pins"));
  for (int i = 0; i < COUNT_SENSORS; i++) {
    pinMode(sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(sensors[i].shutdown_pin, LOW);
  }
  Serial.println(F("Starting..."));
  Initialize_sensors();

  //set all the status pin
  digitalWrite(LED_IR,LOW);
  digitalWrite(LED_RECOG,LOW);
  digitalWrite(LED_NOTRECOG,LOW);
  digitalWrite(ESP_enabled,HIGH);//setting that esp is ready to go

    xTaskCreatePinnedToCore(
    task1,              // Function to be executed
    "Task1",            // Name of the task
    20000,              // Stack size
    NULL,               // Parameters
    1,                  // Priority
    &Task1,             // Task handle
    1 // Core number (0 for Unicore, 1 for Dual-core)
  );

    xTaskCreatePinnedToCore(
    task2,              // Function to be executed
    "Task2",            // Name of the task
    20000,              // Stack size
    NULL,               // Parameters
    1,                  // Priority
    &Task2,             // Task handle
    0// Core number (0 for Unicore, 1 for Dual-core)
  );

}

//====================================================================
// loop
//====================================================================
void loop() {
vTaskDelay(1000 / portTICK_PERIOD_MS);
}