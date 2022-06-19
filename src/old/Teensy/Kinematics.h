#include "math.h"

#define PI 3.14159265

const float upper_leg_length = 28;
const float lower_leg_length = 28;

const int coxa = 0;
const float femur = 28;
const float tibia = 28; // measurements in cm


class KinematicModel{
public:
    float domainCheck(float &D){
        if(D > 1 || D < -1){
            if(D > 1){
                D = 0.99;
                return D;
            }
            else if(D < -1){
                D = -0.99;
                return D;
            }
        }
        else{
            return D;
        }
    }

    float LegKinematicModel(float x, float y, float z, int sign, float &upper_angle, float &lower_angle, float &hip_angle){
        float D = (pow(y, 2) + pow(z, 2) - pow(coxa, 2) + pow(x, 2) - pow(femur, 2) - pow(tibia, 2))/(2 * tibia * femur);
        D = domainCheck(D);
        lower_angle = (sign * atan2(sqrt(1 - pow(D, 2)), D)) * 180/PI;
        hip_angle = (-atan2(-z, y) - atan2((sqrt(pow(y, 2) + pow(-z, 2) - pow(coxa, 2))), -coxa)) * 180/PI;
        upper_angle = (atan2(x, sqrt(pow(y, 2) + pow(z, 2) - pow(coxa, 2)) - atan2(tibia * sin(lower_angle), femur + tibia * cos(lower_angle)))) * 180/PI;
    }

    float angle2Rot(float angle){
        float rot = angle * 10/360;
        return rot;
    }
};