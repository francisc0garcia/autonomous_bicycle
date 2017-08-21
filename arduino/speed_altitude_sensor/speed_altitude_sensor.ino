// include and external libraries
#include <Wire.h>
#include <TimerOne.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_BMP280.h>
#define USE_USBCON
#include <ros.h>
#include <std_msgs/Float32.h>

// ROS variables
//NodeHandle_: # subscribers, #  publishers, input buffer(bytes), output buffer)(bytes)
ros::NodeHandle_<ArduinoHardware, 0, 2, 100, 100> nh;

std_msgs::Float32 altitude_msg;
std_msgs::Float32 velocity_msg;

ros::Publisher pub_altitude("altitude", &altitude_msg);
ros::Publisher pub_velocity("velocity", &velocity_msg);

long publisher_timer = 0;
int period = 10; // ms

// Adafruit Barometer parameters
bool use_bmp180 = false;
bool use_bmp280 = true;
Adafruit_BMP280 bme; // I2C
Adafruit_BMP085 bmp;
float altitude = 0.00;
float temperature = 0.00;
// search daily pressure: http://weather.unisys.com/forecast.php?Name=Frankfurt
double default_pressure_sea_level = 1012.90;

// Speedmeter variables
#define reed_pin A0       // pin connected to reed switch
float radius = 0.00030;   // the wheel radius, in kilometers
float velocityGain = 0.23; // Obtained with experiments using GPS velocity
int time = 0;
int max_time = 3000; // max delay before v = 0
int reedVal;
int maxDebounceTicks = 100;
int currentDebounceTicks;
float circumference;
float velocity = 0.00;
volatile float max_velocity = 40.00; // (m/s)
volatile float velocity_tmp = 0.00; // use volatile for shared variables

void setup() {
    delay(50);

    // Init ROS publishers
    nh.initNode();
    nh.advertise(pub_altitude);
    nh.advertise(pub_velocity);
    delay(50);

    // Init Adafruit barometer
    if(use_bmp280){
        bme.begin();
    }

    if(use_bmp180){
        bmp.begin();
    }

    delay(50);

    currentDebounceTicks = maxDebounceTicks;
    circumference = 2 * 3.1416 * radius * velocityGain;

    pinMode(reed_pin, INPUT);

    Timer1.initialize(1000);
    Timer1.attachInterrupt(compute_velocity);

    delay(50);
}

// Compute velocity and distance based on reed switch
void compute_velocity(void)
{
    reedVal = digitalRead(reed_pin);
    if (reedVal) {
        // wait the given number of ticks, before calculating the velocity
        if (currentDebounceTicks == 0) {
            // circumference in kilometers, time in hours
            velocity_tmp = circumference/(float(time)/1000/3600);

            if(velocity_tmp > max_velocity){
                velocity_tmp = max_velocity;
            }

            time = 0;
            currentDebounceTicks = maxDebounceTicks;
        } else {
            if (currentDebounceTicks > 0) {
                currentDebounceTicks -= 1;
            }
        }
    } else {
        if (currentDebounceTicks > 0) {
            currentDebounceTicks -= 1;
        }
    }

    if (time > max_time) {
        // set velocity to 0 when tire is still for 2 seconds
        velocity_tmp = 0;
    } else {
        time += 1;
    }
}

void loop() {
    if (millis() > publisher_timer) {
        // BMP180
        if(use_bmp180){
            altitude = bmp.readAltitude(default_pressure_sea_level);
        }

        // BMP280
        if(use_bmp280){
            //altitude = bme.readAltitude(default_pressure_sea_level);
            altitude = bme.readAltitude(default_pressure_sea_level);
        }
        //altitude = bmp.readAltitude();
        //temperature = bmp.readTemperature();
        //pressure = bmp.readPressure();

        // Update values for velocity and distance
        noInterrupts();
        velocity = velocity_tmp;
        interrupts();

        // Publish ROS messages
        altitude_msg.data = altitude;
        pub_altitude.publish(&altitude_msg);

        velocity_msg.data = velocity;
        pub_velocity.publish(&velocity_msg);

        publisher_timer = millis() + period;
    }

    nh.spinOnce();
}