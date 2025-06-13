#include <Arduino.h>
#include "RobotArm.h"

RobotArm arm;

void setup() {
    Serial.begin(115200);
    Serial.println("=== RobotArm 6DOF Full Example ===");

    // ---- Robot Structure ----
    arm.addJoint(-PI, PI, 0.0f);
    arm.addJoint(-PI/2.0f, PI/2.0f, 0.0f);
    arm.addJoint(-PI/2.0f, PI/2.0f, 0.0f);
    arm.addJoint(-PI, PI, 0.0f);
    arm.addJoint(-PI, PI, 0.0f);
    arm.addJoint(-PI, PI, 0.0f);

    arm.addLink(0.5f, 0.0f, 0.4f);
    arm.addLink(0.3f, 0.0f, 0.2f);
    arm.addLink(0.2f, 0.0f, 0.1f);
    arm.addLink(0.15f, 0.0f, 0.1f);
    arm.addLink(0.1f, 0.0f, 0.05f);
    arm.addLink(0.05f, 0.0f, 0.02f);

    // ---- Initial Pose ----
    arm.setJointAngle(0, 0.0f);
    arm.setJointAngle(1, -PI/6.0f);
    arm.setJointAngle(2, PI/6.0f);
    arm.setJointAngle(3, 0.0f);
    arm.setJointAngle(4, 0.0f);
    arm.setJointAngle(5, 0.0f);

    Serial.println("Initial Joint Angles:");
    for (size_t i = 0; i < arm.getNumJoints(); ++i) {
        float angle_deg = arm.getJointAngle(i) * 180.0f / PI;
        Serial.print("S");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(angle_deg);
        Serial.println(" deg");
    }

    // ---- IK Parameters ----
    arm.setMaxIterations(200);
    arm.setTolerance(0.005f);
    arm.setLearningRate(0.2f);

    // ---- Target Position + Orientation ----
    float target_x = 0.9f;
    float target_y = 0.3f;
    float target_z = 0.6f;
    float target_roll = 0.0f;   // keep orientation simple for now
    float target_pitch = 0.0f;
    float target_yaw = 0.0f;

    // ---- Initial Guess ----
    std::vector<float> initial_guess(arm.getNumJoints());
    for (size_t i = 0; i < arm.getNumJoints(); ++i) {
        initial_guess[i] = arm.getJointAngle(i);  // Use current pose as initial guess
    }

    // ---- Compute IK ----
    Serial.println("Computing IK...");
    arm.computeIK(target_x, target_y, target_z,
                  target_roll, target_pitch, target_yaw,
                  initial_guess);

    // ---- Result ----
    Serial.println("Result Joint Angles (degrees):");
    for (size_t i = 0; i < arm.getNumJoints(); ++i) {
        float angle_deg = arm.getJointAngle(i) * 180.0f / PI;
        Serial.print("S");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(angle_deg);
        Serial.println(" deg");
    }

    Serial.println("=== End of Example ===");
}

void loop() {
    // Nothing to do here
}
