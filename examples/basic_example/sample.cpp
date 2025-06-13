#include <Arduino.h>
#include "RobotArm.h"

RobotArm arm;

void setup() {
    Serial.begin(115200);
    Serial.println("RobotArm Example with RobotArm class");

    arm.addJoint(-PI, PI, 0.0f);
    arm.addJoint(-PI/2.0f, PI/2.0f, 0.0f);

    arm.addLink(0.5f, 0.0f, 0.4f);
    arm.addLink(0.3f, 0.0f, 0.2f);

    std::vector<float> initial_guess(arm.getNumJoints(), 0.0f);

    float target_x = 0.6f;
    float target_y = 0.2f;
    float target_z = 0.5f;

    arm.computeIK(target_x, target_y, target_z, initial_guess);

    for (size_t i = 0; i < arm.getNumJoints(); ++i) {
        float angle_rad = arm.getJointAngle(i);
        float angle_deg = angle_rad * 180.0f / PI;
        Serial.print("S");
        Serial.print(i);
        Serial.print(":");
        Serial.println(angle_deg);
    }

    Serial.println("Finished.");
}

void loop() {}
