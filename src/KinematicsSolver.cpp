#include "KinematicsSolver.h"
#include <Arduino.h>
#include <math.h>

KinematicsSolver::KinematicsSolver(std::vector<Joint>& joints,
                                   const std::vector<Link>& links)
    : joints_(joints), links_(links),
      max_iterations_(100),
      tolerance_(0.01f),
      learning_rate_(0.1f) {}

void KinematicsSolver::setMaxIterations(int max_iterations) {
    max_iterations_ = max_iterations;
}

void KinematicsSolver::setTolerance(float tolerance) {
    tolerance_ = tolerance;
}

void KinematicsSolver::setLearningRate(float learning_rate) {
    learning_rate_ = learning_rate;
}

void KinematicsSolver::computeIK(float target_x, float target_y, float target_z,
                                 float target_roll, float target_pitch, float target_yaw,
                                 const std::vector<float>& initial_guess) {
    std::vector<float> joint_angles = initial_guess;

    for (int iter = 0; iter < max_iterations_; ++iter) {
        float current_x, current_y, current_z;
        float current_roll, current_pitch, current_yaw;

        forwardKinematics(joint_angles, current_x, current_y, current_z,
                          current_roll, current_pitch, current_yaw);

        float error[6] = {
            target_x - current_x,
            target_y - current_y,
            target_z - current_z,
            target_roll - current_roll,
            target_pitch - current_pitch,
            target_yaw - current_yaw
        };

        float error_norm = 0.0f;
        for (int i = 0; i < 6; ++i) {
            error_norm += error[i] * error[i];
        }
        error_norm = sqrt(error_norm);

        if (error_norm < tolerance_) {
            Serial.print("IK converged in ");
            Serial.print(iter);
            Serial.println(" iterations.");
            break;
        }

        std::vector<std::vector<float>> J(6, std::vector<float>(joint_angles.size(), 0.0f));
        computeJacobian(joint_angles, J);

        // Jacobian Transpose update
        for (size_t i = 0; i < joint_angles.size(); ++i) {
            float delta_theta = 0.0f;
            for (int j = 0; j < 6; ++j) {
                delta_theta += J[j][i] * error[j];
            }
            delta_theta *= learning_rate_;

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
                                         float& x, float& y, float& z,
                                         float& roll, float& pitch, float& yaw) const {
    Matrix4x4 T;
    T.setIdentity();

    size_t n = std::min(joint_angles.size(), links_.size());
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

        T *= Ti;
    }

    x = T.data[0][3];
    y = T.data[1][3];
    z = T.data[2][3];

    T.getRPY(roll, pitch, yaw);
}

void KinematicsSolver::computeJacobian(const std::vector<float>& joint_angles,
                                       std::vector<std::vector<float>>& J) const {
    const float delta = 0.0001f;

    float base_x, base_y, base_z;
    float base_roll, base_pitch, base_yaw;

    forwardKinematics(joint_angles, base_x, base_y, base_z,
                      base_roll, base_pitch, base_yaw);

    for (size_t i = 0; i < joint_angles.size(); ++i) {
        std::vector<float> temp_angles = joint_angles;
        temp_angles[i] += delta;

        float dx, dy, dz;
        float d_roll, d_pitch, d_yaw;

        forwardKinematics(temp_angles, dx, dy, dz,
                          d_roll, d_pitch, d_yaw);

        J[0][i] = (dx - base_x) / delta;
        J[1][i] = (dy - base_y) / delta;
        J[2][i] = (dz - base_z) / delta;
        J[3][i] = (d_roll - base_roll) / delta;
        J[4][i] = (d_pitch - base_pitch) / delta;
        J[5][i] = (d_yaw - base_yaw) / delta;
    }
}

float KinematicsSolver::clamp(float val, float min_val, float max_val) const {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}
