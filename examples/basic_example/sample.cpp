#include <Arduino.h>
#include "RobotArm.h"

RobotArm arm;

void setup() {
    Serial.begin(9600);
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
    float target_roll = 0.0f;   // for simplicity
    float target_pitch = 0.0f;
    float target_yaw = 0.0f;

    // ---- Compute IK ----
    Serial.println("Computing IK...");
    unsigned long start_time = micros();

    arm.computeIK(target_x, target_y, target_z,
                  target_roll, target_pitch, target_yaw);

    unsigned long end_time = micros();
    unsigned long elapsed_time_us = end_time - start_time;

    Serial.print("IK Computation Time: ");
    Serial.print(elapsed_time_us / 1000.0f);
    Serial.println(" ms");

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

    // ---- IK Precision Check ----
    Serial.println("=== IK Precision Check ===");

    float result_x, result_y, result_z;
    float result_roll, result_pitch, result_yaw;

    arm.getEndEffectorPose(result_x, result_y, result_z,
                           result_roll, result_pitch, result_yaw);

    Serial.print("Target X: "); Serial.print(target_x); Serial.print(" , Result X: "); Serial.print(result_x); Serial.print(" , Error: "); Serial.println(fabs(target_x - result_x));
    Serial.print("Target Y: "); Serial.print(target_y); Serial.print(" , Result Y: "); Serial.print(result_y); Serial.print(" , Error: "); Serial.println(fabs(target_y - result_y));
    Serial.print("Target Z: "); Serial.print(target_z); Serial.print(" , Result Z: "); Serial.print(result_z); Serial.print(" , Error: "); Serial.println(fabs(target_z - result_z));

    Serial.print("Target Roll: "); Serial.print(target_roll); Serial.print(" , Result Roll: "); Serial.print(result_roll); Serial.print(" , Error: "); Serial.println(fabs(target_roll - result_roll));
    Serial.print("Target Pitch: "); Serial.print(target_pitch); Serial.print(" , Result Pitch: "); Serial.print(result_pitch); Serial.print(" , Error: "); Serial.println(fabs(target_pitch - result_pitch));
    Serial.print("Target Yaw: "); Serial.print(target_yaw); Serial.print(" , Result Yaw: "); Serial.print(result_yaw); Serial.print(" , Error: "); Serial.println(fabs(target_yaw - result_yaw));

    Serial.println("=== End of Example ===");
}

void loop() {
    // Nothing to do here
}
