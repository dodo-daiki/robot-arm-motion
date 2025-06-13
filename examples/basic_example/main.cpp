#include <Arduino.h>
#include <RobotArm.h>

RobotArm arm;

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Set link lengths
    for (int i = 0; i < arm.getModel().getNumLinks(); ++i) {
        arm.getModel().setLinkLength(i, 0.1);
    }

    // Set initial joint angles
    double initialAngles[6] = {0, 0, 0, 0, 0, 0};
    arm.setJointAngles(initialAngles);

    // Forward Kinematics
    Pose pose = arm.forwardKinematics();
    Serial.print("FK Position: ");
    Serial.print(pose.position.x()); Serial.print(", ");
    Serial.print(pose.position.y()); Serial.print(", ");
    Serial.println(pose.position.z());

    // Inverse Kinematics target
    Pose targetPose;
    targetPose.position << 0.2, 0.0, 0.0;

    bool success = arm.inverseKinematics(targetPose);
    Serial.println(success ? "IK Success" : "IK Failed");
}

void loop() {
    // Nothing
}
