//
// Created by harut on 10/2/2021.
//

#ifndef QUADRUPED_ROBOTKINEMATICMODEL_H
#define QUADRUPED_ROBOTKINEMATICMODEL_H

#include "Kinematics.h"
#include "math.h"

#define PI = 3.14159265

float LFx = 0.00;
float LFy = 0.00;

float RFx = 0.00;
float RFy = 0.00;

float LRx = 0.00;
float LRy = 0.00;

float RRx = 0.00;
float RRy = 0.00;

int command = 0;

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

class robotKinmatics{
public:
    float coordinateCalculation(&LFx, &LFy,
                                &RFx, &RFy,
                                &LRx, &LRy,
                                &RRx, &RRy){
        if (command == 0){ // IDLE STATE
            float LFx = 0.00;
            float LFy = -0.25;

            float RFx = 0.00;
            float RFy = -0.25;

            float LRx = 0.00;
            float LRy = -0.25;

            float RRx = 0.00;
            float RRy = -0.25;
        }
        else if (command == 1){
            bezier(LFx, LFy);
            bezier(RFx, LFy);
            bezier(LFx, LFy);
            bezier(LFx, LFy);
        }
    }
};

#endif //QUADRUPED_KINEMATICMODEL_H
#endif //QUADRUPED_ROBOTKINEMATICMODEL_H
