#include "RobotKinematics.h"
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>

RobotKinematics::RobotKinematics(const RobotConfig& config)
    : _config(config), _accel(12345) // Initialize with sensor ID
{
    // Initialize current angles (default: all 0)
    for (int i = 0; i < 6; ++i) {
        _currentAngles.angles[i] = 0.0f;
    }
    
    // Initialize accelerometer
    if (!_accel.begin()) {
        Serial.println(F("ADXL345 not found, check wiring!"));
        // Continue without accelerometer functionality
    } else {
        Serial.println(F("ADXL345 accelerometer initialized"));
        
        // Configure accelerometer
        _accel.setRange(ADXL345_RANGE_4_G);         // Set range to ±4g
        _accel.setDataRate(ADXL345_DATARATE_100_HZ); // Set data rate
    }
    
    // Calculate initial current pose
    _currentPose = forwardKinematics(_currentAngles);
}

void RobotKinematics::setCurrentJointAngles(const JointAngles& angles) {
    // Set current joint angles and recalculate pose
    _currentAngles = angles;
    _currentPose = forwardKinematics(_currentAngles);
}

JointAngles RobotKinematics::getCurrentJointAngles() const {
    return _currentAngles;
}

CartesianPose RobotKinematics::getCurrentPose() const {
    return _currentPose;
}

void RobotKinematics::setToolOffset(float x, float y, float z) {
    _config.toolOffsetX = x;
    _config.toolOffsetY = y;
    _config.toolOffsetZ = z;
    // Recalculate current pose with new offsets
    _currentPose = forwardKinematics(_currentAngles);
}

CartesianPose RobotKinematics::forwardKinematics(const JointAngles& angles) {
    // Debug test point 1: Start Forward-Kinematics
    
    // Use Eigen for matrix calculations
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();  // Transformation matrix Base->Tool
    
    // Transformation up to end effector (Frame 5, without Tool-Offset)
    for (int i = 0; i < 6; ++i) {
        // DH-Parameters from Config
        float a = _config.dhParams[i].a;
        float d = _config.dhParams[i].d;
        float alpha = _config.dhParams[i].alpha;
        float theta_offset = _config.dhParams[i].theta;
        
        // Joint angle + Offset
        float theta = angles.angles[i] + theta_offset;
        float cth = cos(theta);
        float sth = sin(theta);
        float ca = cos(alpha);
        float sa = sin(alpha);
        
        // Denavit-Hartenberg transformation matrix for this joint
        Eigen::Matrix4f A;
        A << cth, -sth * ca,  sth * sa,  a * cth,
             sth,  cth * ca, -cth * sa,  a * sth,
             0.0,       sa,       ca,       d,
             0.0,      0.0,      0.0,     1.0;
        
        // Multiply cumulative transformation
        T = T * A;
    }
    
    // At the end of T: Base -> last joint (gripper flange)
    // Add Tool-Offset in world coordinates
    Eigen::Vector3f toolOffsetVec(_config.toolOffsetX, _config.toolOffsetY, _config.toolOffsetZ);
    Eigen::Vector3f pos_flange = T.block<3,1>(0,3);         // Translation part (x,y,z) from T
    Eigen::Matrix3f R_flange = T.block<3,3>(0,0);           // Rotation matrix part from T
    Eigen::Vector3f pos_tool = pos_flange + R_flange * toolOffsetVec;  // Tool tip position

    // Euler angles (ZYX) from the total rotation matrix (R_total = R_flange, as Tool-Offset is pure translation)
    // R_flange corresponds to the orientation of the tool (last joint defines orientation of tool axis).
    float yaw   = atan2(R_flange(1,0), R_flange(0,0));
    float pitch = atan2(-R_flange(2,0), sqrt(pow(R_flange(2,1),2) + pow(R_flange(2,2),2)));
    float roll  = atan2(R_flange(2,1), R_flange(2,2));

    // Package result in CartesianPose structure
    CartesianPose result;
    result.x = pos_tool(0);
    result.y = pos_tool(1);
    result.z = pos_tool(2);
    result.yaw = yaw;
    result.pitch = pitch;
    result.roll = roll;

    // Debug test point 1: End Forward-Kinematics
    
    return result;
}

bool RobotKinematics::inverseKinematics(const CartesianPose& targetPose, JointAngles& outAngles) {
    // Debug test point 0: Start Inverse-Kinematics
    
    // Start with current angle state (start of iteration)
    Eigen::Matrix<float, 6, 1> theta;
    for (int i = 0; i < 6; ++i) {
        theta(i) = _currentAngles.angles[i];
    }
    
    const int MAX_ITER = 100;
    const float POS_EPS = 0.5f;        // Position tolerance (mm)
    const float ORI_EPS = 0.01f;       // Orientation tolerance (rad ~0.57°)
    const float LAMBDA = 0.5f;         // Damping factor for convergence
    
    bool success = false;
    
    for (int iter = 0; iter < MAX_ITER; ++iter) {
        // Calculate forward kinematics for current theta estimate
        JointAngles guessAngles;
        for (int j = 0; j < 6; ++j) guessAngles.angles[j] = theta(j);
        CartesianPose currentPose = forwardKinematics(guessAngles);
        
        // Error vectors (position and orientation)
        Eigen::Vector3f pos_err;
        pos_err << targetPose.x - currentPose.x,
                   targetPose.y - currentPose.y,
                   targetPose.z - currentPose.z;
        
        // Orientation error as rotation vector (axis*angle)
        // Calculate rotation matrix for current pose and target pose
        float cy = cos(targetPose.yaw),   sy = sin(targetPose.yaw);
        float cp = cos(targetPose.pitch), sp = sin(targetPose.pitch);
        float cr = cos(targetPose.roll),  sr = sin(targetPose.roll);
        
        // Target rotation matrix (ZYX from yaw,pitch,roll)
        Eigen::Matrix3f R_target;
        R_target << cy*cp,    cy*sp*sr - sy*cr,    cy*sp*cr + sy*sr,
                    sy*cp,    sy*sp*sr + cy*cr,    sy*sp*cr - cy*sr,
                    -sp,      cp*sr,               cp*cr;
        
        // Current rotation matrix from pose (ZYX calculated, now invert to get matrix)
        float cy_c = cos(currentPose.yaw),   sy_c = sin(currentPose.yaw);
        float cp_c = cos(currentPose.pitch), sp_c = sin(currentPose.pitch);
        float cr_c = cos(currentPose.roll),  sr_c = sin(currentPose.roll);
        
        Eigen::Matrix3f R_current;
        R_current << cy_c*cp_c,    cy_c*sp_c*sr_c - sy_c*cr_c,    cy_c*sp_c*cr_c + sy_c*sr_c,
                     sy_c*cp_c,    sy_c*sp_c*sr_c + cy_c*cr_c,    sy_c*sp_c*cr_c - sy_c*sr_c,
                     -sp_c,        cp_c*sr_c,                    cp_c*cr_c;
        
        // Rotation error R_err = R_target * R_current^T
        Eigen::Matrix3f R_err = R_target * R_current.transpose();
        
        // Extract axis-angle from R_err
        float angle_err = acosf(fminf(1.0f, ((float)R_err.trace() - 1.0f) / 2.0f));  // Angle
        
        Eigen::Vector3f axis_err;
        if (angle_err < 1e-6f) {
            axis_err << 0, 0, 0;
        } else {
            axis_err(0) = R_err(2,1) - R_err(1,2);
            axis_err(1) = R_err(0,2) - R_err(2,0);
            axis_err(2) = R_err(1,0) - R_err(0,1);
            axis_err /= (2.0f * sin(angle_err));
        }
        
        Eigen::Vector3f ori_err = angle_err * axis_err;  // Orientation error vector (in world CS)
        
        // Check if error is small enough to terminate
        if (pos_err.norm() < POS_EPS && ori_err.norm() < ORI_EPS) {
            success = true;
            break;
        }
        
        // Jacobian matrix J (6x6)
        Eigen::Matrix<float, 6, 6> J;
        
        // For Jacobian: Calculate intermediate positions and axis directions
        // Build transformation matrix incrementally
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        
        // Base parameters
        Eigen::Vector3f origin_prev = Eigen::Vector3f(0,0,0);    // O_{-1}: Base origin
        Eigen::Vector3f z_prev = Eigen::Vector3f(0,0,1);         // z_{-1}: Base z-axis (assumed vertical)
        
        for (int j = 0; j < 6; ++j) {
            // Axis j: z_prev (orientation of the (j)th joint axis in world coordinates)
            // Origin vector of joint j: origin_prev
            // Position of tool relative to origin_prev:
            Eigen::Vector3f origin_tool;
            origin_tool << currentPose.x, currentPose.y, currentPose.z;
            
            // Linear component: z_prev x (P_tool - O_prev)
            Eigen::Vector3f lin = z_prev.cross(origin_tool - origin_prev);
            
            // Rotational component: z-axis
            Eigen::Vector3f rot = z_prev;
            
            // Insert column
            J.block<3,1>(0,j) = lin;    // Linear part (position Jacobian)
            J.block<3,1>(3,j) = rot;    // Angular part (orientation Jacobian)
            
            // Now update for next joint:
            // DH-Parameters from Config
            float a = _config.dhParams[j].a;
            float d = _config.dhParams[j].d;
            float alpha = _config.dhParams[j].alpha;
            float theta_offset = _config.dhParams[j].theta;
            
            // Joint angle + Offset
            float theta_j = theta(j) + theta_offset;
            float cth = cos(theta_j);
            float sth = sin(theta_j);
            float ca = cos(alpha);
            float sa = sin(alpha);
            
            // Denavit-Hartenberg transformation matrix for this joint
            Eigen::Matrix4f A;
            A << cth, -sth * ca,  sth * sa,  a * cth,
                 sth,  cth * ca, -cth * sa,  a * sth,
                 0.0,       sa,       ca,       d,
                 0.0,      0.0,      0.0,     1.0;
            
            T = T * A;
            
            // Extract new origin and z-axis for the next joint
            origin_prev = T.block<3,1>(0,3);       // Translation part of T
            z_prev = T.block<3,3>(0,0).col(2);     // Z-axis is 3rd column of rotation matrix
        }
        
        // Error vectors combined
        Eigen::Matrix<float, 6, 1> error;
        error.block<3,1>(0,0) = pos_err;
        error.block<3,1>(3,0) = ori_err;
        
        // Pseudoinverse with damping (robust)
        Eigen::Matrix<float, 6, 6> J_pinv = J.transpose() *
            (J * J.transpose() + LAMBDA * LAMBDA * Eigen::Matrix<float, 6, 6>::Identity()).inverse();
        
        // Update theta estimate
        theta = theta + J_pinv * error;
        
        // Apply joint limits
        for (int j = 0; j < 6; ++j) {
            // Normalize to [-pi, pi] and then check limits
            theta(j) = normalizeAngle(theta(j));
            
            if (theta(j) < _config.jointMin[j]) {
                theta(j) = _config.jointMin[j];
            }
            if (theta(j) > _config.jointMax[j]) {
                theta(j) = _config.jointMax[j];
            }
        }
    }
    
    // Return result
    if (success) {
        for (int i = 0; i < 6; ++i) {
            outAngles.angles[i] = theta(i);
        }
    }
    
    // Debug test point 0: End Inverse-Kinematics
    
    return success;
}

bool RobotKinematics::isPoseReachable(const CartesianPose& pose) {
    // Calculate minimal required arm length r from base to target
    float r = sqrt(pose.x*pose.x + pose.y*pose.y + pose.z*pose.z);
    
    // Get length of two main arm segments from DH parameters
    float a1 = _config.dhParams[1].a; // Shoulder to elbow length
    float a2 = _config.dhParams[2].a; // Elbow to wrist length
    
    // Simple check - if distance exceeds maximum arm extension
    if (r > (a1 + a2)) {
        return false;
    }
    
    // Check if too close to base (joints would collide with base)
    float d0 = _config.dhParams[0].d; // Base height
    if (sqrt(pose.x*pose.x + pose.y*pose.y) < 10.0f && pose.z < d0) {
        return false;
    }
    
    // Try inverse kinematics to see if solution exists
    JointAngles testAngles;
    bool hasIkSolution = inverseKinematics(pose, testAngles);
    
    return hasIkSolution;
}

float RobotKinematics::normalizeAngle(float angle) {
    // Normalize angle to [-PI, PI]
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

AccelData RobotKinematics::getToolAcceleration() {
    AccelData result = {0}; // Initialize all fields to zero
    
    // Read accelerometer
    sensors_event_t event;
    if (_accel.getEvent(&event)) {
        // Raw readings in m/s²
        result.x_ms2 = event.acceleration.x;
        result.y_ms2 = event.acceleration.y;
        result.z_ms2 = event.acceleration.z;
        
        // Convert to G-forces (1G = 9.80665 m/s²)
        const float g = 9.80665f;
        result.x_g = result.x_ms2 / g;
        result.y_g = result.y_ms2 / g;
        result.z_g = result.z_ms2 / g;
        
        // Copy to standard fields
        result.x = result.x_ms2;
        result.y = result.y_ms2;
        result.z = result.z_ms2;
        
        // Calculate pitch and roll (simplified - assumes X/Y plane is level when idle)
        result.pitch = atan2(result.x_g, sqrt(result.y_g * result.y_g + result.z_g * result.z_g)) * 180.0 / M_PI;
        result.roll = atan2(result.y_g, result.z_g) * 180.0 / M_PI;
    }
    
    return result;
}
