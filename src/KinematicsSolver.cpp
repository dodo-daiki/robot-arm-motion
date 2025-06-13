#include "KinematicsSolver.h"
#include <Arduino.h>
#include <math.h>

KinematicsSolver::KinematicsSolver(std::vector<Joint>& joints,
                                   const std::vector<Link>& links)
    : joints_(joints), links_(links) {}

void KinematicsSolver::computeIK(float target_x, float target_y, float target_z,
                                 const std::vector<float>& initial_guess) {
    std::vector<float> joint_angles = initial_guess;
    const int max_iterations = 100;
    const float tolerance = 0.01;
    const float learning_rate = 0.1;

    for (int iter = 0; iter < max_iterations; ++iter) {
        float current_x, current_y, current_z;
        forwardKinematics(joint_angles, current_x, current_y, current_z);

        float error_x = target_x - current_x;
        float error_y = target_y - current_y;
        float error_z = target_z - current_z;

        float error_norm = sqrt(error_x * error_x + error_y * error_y + error_z * error_z);
        if (error_norm < tolerance) {
            Serial.print("IK converged in ");
            Serial.print(iter);
            Serial.println(" iterations.");
            break;
        }

        float J[3][3];
        computeJacobian(joint_angles, J);

        // Jacobian Transpose update
        for (size_t i = 0; i < joint_angles.size(); ++i) {
            float delta_theta = learning_rate * (
                J[0][i] * error_x +
                J[1][i] * error_y +
                J[2][i] * error_z
            );

            joint_angles[i] += delta_theta;
            joint_angles[i] = clamp(joint_angles[i],
                                    joints_[i].getLimitMin(),
                                    joints_[i].getLimitMax());
        }
    }

    // 最終結果を joints_ に反映
    for (size_t i = 0; i < joints_.size(); ++i) {
        joints_[i].setCurrentValue(joint_angles[i]);
    }
}

void KinematicsSolver::forwardKinematics(const std::vector<float>& joint_angles,
                                         float& x, float& y, float& z) const {
    Matrix4x4 T;
    T.setIdentity();

    size_t n = min(joint_angles.size(), links_.size());
    for (size_t i = 0; i < n; ++i) {
        float theta = joint_angles[i] + joints_[i].getThetaOffset();
        float a = links_[i].getA();
        float alpha = links_[i].getAlpha();
        float d = links_[i].getD();

        Matrix4x4 Ti;
        Ti.setIdentity();

        Ti.data[0][0] = cos(theta);
        Ti.data[0][1] = -sin(theta) * cos(alpha);
        Ti.data[0][2] = sin(theta) * sin(alpha);
        Ti.data[0][3] = a * cos(theta);

        Ti.data[1][0] = sin(theta);
        Ti.data[1][1] = cos(theta) * cos(alpha);
        Ti.data[1][2] = -cos(theta) * sin(alpha);
        Ti.data[1][3] = a * sin(theta);

        Ti.data[2][0] = 0.0f;
        Ti.data[2][1] = sin(alpha);
        Ti.data[2][2] = cos(alpha);
        Ti.data[2][3] = d;

        Ti.data[3][0] = 0.0f;
        Ti.data[3][1] = 0.0f;
        Ti.data[3][2] = 0.0f;
        Ti.data[3][3] = 1.0f;

        T.multiply(Ti);
    }

    // Extract end effector position
    x = T.data[0][3];
    y = T.data[1][3];
    z = T.data[2][3];
}

void KinematicsSolver::computeJacobian(const std::vector<float>& joint_angles,
                                       float J[3][3]) const {
    const float delta = 0.0001;

    float base_x, base_y, base_z;
    forwardKinematics(joint_angles, base_x, base_y, base_z);

    for (size_t i = 0; i < joint_angles.size(); ++i) {
        std::vector<float> temp_angles = joint_angles;
        temp_angles[i] += delta;

        float dx, dy, dz;
        forwardKinematics(temp_angles, dx, dy, dz);

        J[0][i] = (dx - base_x) / delta;
        J[1][i] = (dy - base_y) / delta;
        J[2][i] = (dz - base_z) / delta;
    }
}

float KinematicsSolver::clamp(float val, float min_val, float max_val) const {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}
