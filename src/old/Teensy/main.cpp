#include "Arduino.h"
#include "ODriveArduino.h"
#include "SoftwareSerial.h"
#include "HardwareSerial.h"
#include "math.h"
#include "iostream"
#include "string.h"
#include "vector"
#include "Kinematics.h"

using namespace std;

template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// robot measurements
const float upper_bone = 0.3f;
const float lower_bone = 0.3f;

int factorial(int num){
    return gamma(num+1);
}

float function_f(int n, int k) {
    return factorial(n)/(factorial(k) * factorial(n-k));
}

float function_b(float t, int k, float point) {
    int n = 11;
    return point * function_f(n, k) * pow(t, k) * pow(1-t, n-k);
}

void bezier(float &x_coordinate, float &y_coordinate, float T) {
    // HW: static in function
    static int bezier_x [] = {-10, -15, -25, -25, -25, 0, 0, 0, 25, 25, 20, 10};
    static int bezier_y [] = {-35, -35, -20, -20, -20, -25, -25, -15, -15, -15, -35, -35};

    x_coordinate = y_coordinate = 0.0;
    for (size_t i = 0; i < 12; i++){
        x_coordinate += function_b(T, i, bezier_x[i]);
        y_coordinate += function_b(T, i, bezier_y[i]);
    }
}

struct ODrive {
    ODriveArduino   odrive;
    HardwareSerial& serial;

    ODrive(HardwareSerial& serial) : odrive(serial), serial(serial) {}

    bool run_state(int axis, int requested_state, bool wait_for_idle, float timeout = 10.0f) {
        serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
        if(!odrive.run_state(axis, requested_state, wait_for_idle, timeout)) {
            return;
        }
    }
};

std::vector<ODrive> odriveList;
KinematicModel      kinematc;

struct Coordinate {
    float x;
    float y;
    float z;

    Coordinate() : x(0.0f), y(0.0f), z(0.0f) {}
};

std::vector<Coordinate> leg_coordinates;

void setup() {
    // Initialize ODrive
    odriveList.push_back(ODrive(Serial1));
    odriveList.push_back(ODrive(Serial2));
    odriveList.push_back(ODrive(Serial3));
    odriveList.push_back(ODrive(Serial4));
    odriveList.push_back(ODrive(Serial5));
    odriveList.push_back(ODrive(Serial6));

    leg_coordinates.push_back(Coordinate());
    leg_coordinates.push_back(Coordinate());
    leg_coordinates.push_back(Coordinate());
    leg_coordinates.push_back(Coordinate());

    Serial.println("Setting up parameters...");
    for (size_t i = 0; i < odriveList.size(); i++) {
        for (size_t axis = 0; axis < 2; ++axis){

//            leg_coordinates[0].x
            HardwareSerial& serial = odriveList[i].serial;
            serial.begin(115200);

//            serial << ".clear_errors()" << '\n';

            //MOTOR CONFIG
            serial << "w axis" << axis << ".motor.config.current_lim " << 30.0f << '\n';
            serial << "w axis" << axis << ".motor.config.torque_constant " << 0.0827f << '\n';
            serial << "w axis" << axis << ".motor.config.motor_type " << 0.0f << '\n';
            serial << "w axis" << axis << ".motor.config.pole_pairs " << 20.0f << '\n';

            //ENCODER CONFIG
            serial << "w axis" << axis << ".encoder.config.cpr " << 16384.0f << '\n';
            serial << "w axis" << axis << ".encoder.config.mode " << 257.0f << '\n';
            serial << "w axis" << axis << ".encoder.config.calib_range " << 0.05f << '\n';
            serial << "w axis" << axis << ".encoder.config.abs_spi_cs_gpio_pin " << 3 + axis << '\n';
            // ENCODER GPIO PIN 3 -> upper leg, GPIO PIN 4 -> lower leg

            //CONTROLLER CONFIG
            serial << "w axis" << axis << ".controller.config.pos_gain " << 20.0f << '\n';
            serial << "w axis" << axis << ".controller.config.vel_gain " << 0.16f << '\n';
            serial << "w axis" << axis << ".controller.config.vel_integrator_gain " << 0.32f << '\n';
            serial << "w axis" << axis << ".controller.config.vel_limit " << 10.0f << '\n';
            //vel_gain = 0.4
            //pos_gain = 10
            //vel_integrator_gain = 2

//            serial << 'odrv0.reboot()' << '\n';
        }
    }

    Serial.begin(115200);
    while(!Serial);
    Serial.println("Starting....");

    Serial.println("Calibrating");
    for (size_t i = 0; i < odriveList.size(); i++) {
        for (size_t axis = 1; axis < 2; axis++) {
            if (!odriveList[0].run_state(1, ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true)) {
                return;
            }
        }
    }

    Serial.println("Setup done!");
    for (size_t i = 0; i < odriveList.size(); i++) {
        for (size_t axis = 1; axis < 2; axis++) {
            if (!odriveList[0].run_state(1, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, true)) {
                return;
            }
        }
    }
}

void loop() {
// write your code here
    if (Serial.available()) {
        char serialInput = Serial.read();
        Serial.println('a');

        if (serialInput == '0') {
            for (size_t i = 0; i < odriveList.size(); i++) {
                ODriveArduino& odrive0 = odriveList[0].odrive;
                ODriveArduino& odrive1 = odriveList[1].odrive;
                ODriveArduino& odrive2 = odriveList[2].odrive;
                ODriveArduino& odrive3 = odriveList[3].odrive;
                ODriveArduino& odrive4 = odriveList[4].odrive;
                ODriveArduino& odrive5 = odriveList[5].odrive;
                Serial << "Axis" << i << ": Requesting state " << ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL << '\n';
                if (!odrive1.run_state(i, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false)) {
                    return;
                }
                for(float t = 0.00f; t <= 1; t += 0.01){
                    float x_coordinate = 0.00f;
                    float z_coordinate = 0.00f;

                    float hip_rot = 0.00f;
                    float upper_leg_rot = 0.00f;
                    float lower_leg_rot = 0.00f;

                    bezier(x_coordinate, z_coordinate, t);

                    kinematc.LegKinematicModel(x_coordinate, 0, z_coordinate, 1, upper_leg_rot, lower_leg_rot, hip_rot);

                    float hip_angle = kinematc.angle2Rot(hip_rot);
                    float upper_leg_angle = kinematc.angle2Rot(upper_leg_rot);
                    float lower_leg_angle = kinematc.angle2Rot(lower_leg_rot);

                    odrive0.SetPosition(0, upper_leg_angle);
                    odrive0.SetPosition(1, lower_leg_angle);
                }
            }
        }
        if (serialInput == '0') {
            for (size_t i = 0; i < odriveList.size(); i++) {
                ODriveArduino& odrive = odriveList[i].odrive;
                Serial << "Axis" << 1 << ": Requesting state " << ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL << '\n';
                if (!odrive.run_state(1, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false)) {
                    return;
                }
                odrive.SetPosition(1, 1);
            }
        }
        else if (serialInput == '1') {
            for (size_t i = 0; i < odriveList.size(); i++) {
                ODriveArduino& odrive = odriveList[i].odrive;
                Serial << "Axis" << 1 << ": Requesting state " << ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL << '\n';
                if (!odrive.run_state(1, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false)) {
                    return;
                }
                odrive.SetPosition(1, 1);
            }
        }
    }
}