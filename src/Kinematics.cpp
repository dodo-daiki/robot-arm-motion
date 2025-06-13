#include "Kinematics.h"

Kinematics::Kinematics(const Body& body_) : body(body_) {}

Eigen::Affine3d Kinematics::forwardKinematics(const std::vector<double>& jointAngles) const {
    if (jointAngles.size() != body.getNumJoints()) {
        throw std::runtime_error("Joint angle size mismatch with body structure.");
    }

    Eigen::Affine3d T = Eigen::Affine3d::Identity();

    for (size_t i = 0; i < jointAngles.size(); ++i) {
        const Link& link = body.getLink(i);
        const Joint& joint = body.getJoint(i);

        // 固定回転
        T = T * link.getRotation();

        // 関節回転
        T = T * Eigen::AngleAxisd(jointAngles[i], joint.getAxis());

        // オフセット移動
        T = T * Eigen::Translation3d(link.getOffset());
    }

    return T;
}

Eigen::MatrixXd Kinematics::computeJacobian(const std::vector<double>& jointAngles) const {
    size_t n = body.getNumJoints();
    Eigen::MatrixXd J(6, n);

    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    Eigen::Vector3d p_e = forwardKinematics(jointAngles).translation();

    for (size_t i = 0; i < n; ++i) {
        const Link& link = body.getLink(i);
        const Joint& joint = body.getJoint(i);

        T = T * link.getRotation();
        T = T * Eigen::AngleAxisd(jointAngles[i], joint.getAxis());
        T = T * Eigen::Translation3d(link.getOffset());

        Eigen::Vector3d z_i = T.rotation() * joint.getAxis(); // 関節軸方向 (ワールド系)
        Eigen::Vector3d p_i = T.translation();                // 関節位置

        // 位置部分
        J.block<3,1>(0,i) = z_i.cross(p_e - p_i);

        // 姿勢部分
        J.block<3,1>(3,i) = z_i;
    }

    return J;
}

bool Kinematics::inverseKinematics(const Eigen::Affine3d& targetPose, std::vector<double>& jointAngles) const {
    const double epsilon = 1e-4;
    const int maxIter = 100;
    const double alpha = 0.1;

    for (int iter = 0; iter < maxIter; ++iter) {
        Eigen::Affine3d T = forwardKinematics(jointAngles);

        // 位置誤差
        Eigen::Vector3d p_err = targetPose.translation() - T.translation();

        // 姿勢誤差（軸角ベクトル）
        Eigen::Vector3d o_err = MathUtils::rotToAA(T.rotation().transpose() * targetPose.rotation());

        // 誤差ベクトル作成
        Eigen::VectorXd error(6);
        error << p_err, o_err;

        // 収束判定
        if (error.norm() < epsilon) {
            return true;
        }

        Eigen::MatrixXd J = computeJacobian(jointAngles);

        // Damped Least Squares
        Eigen::MatrixXd JT = J.transpose();
        Eigen::MatrixXd JJt = J * JT;
        Eigen::MatrixXd damping = MathUtils::dampingMat(6, 1e-6);
        Eigen::MatrixXd inv = (JJt + damping).inverse();

        Eigen::VectorXd deltaTheta = JT * inv * error;

        // 関節角更新
        for (size_t i = 0; i < jointAngles.size(); ++i) {
            jointAngles[i] += alpha * deltaTheta[i];

            // クランプ
            double minLimit = body.getJoint(i).getMinLimit();
            double maxLimit = body.getJoint(i).getMaxLimit();
            jointAngles[i] = MathUtils::clamp(jointAngles[i], minLimit, maxLimit);

            // 角度正規化（オプション）
            jointAngles[i] = MathUtils::normAngle(jointAngles[i]);
        }
    }

    return false; // 収束せず
}
