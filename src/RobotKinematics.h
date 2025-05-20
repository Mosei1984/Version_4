#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <Arduino.h>
#include <Adafruit_ADXL345_U.h>

// DH-Parameter structure
struct DHParameter {
    float a;      // Link length
    float alpha;  // Link twist
    float d;      // Link offset
    float theta;  // Joint angle (for fixed joints)
};

// Robot configuration structure
struct RobotConfig {
    DHParameter dhParams[6];  // DH parameters for each joint
    float jointMin[6];        // Minimum angle for each joint (radians)
    float jointMax[6];        // Maximum angle for each joint (radians)
    float toolOffsetX;        // Tool offset along X axis
    float toolOffsetY;        // Tool offset along Y axis
    float toolOffsetZ;        // Tool offset along Z axis
};

// Joint angles structure
struct JointAngles {
    float angles[6];
};

// Cartesian pose structure
struct CartesianPose {
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
    float roll;
};

// Accelerometer data structure
struct AccelData {
    float x;      // X acceleration in standard units
    float y;      // Y acceleration in standard units
    float z;      // Z acceleration in standard units
    float x_ms2;  // X acceleration in m/s²
    float y_ms2;  // Y acceleration in m/s²
    float z_ms2;  // Y acceleration in m/s²
    float x_g;    // X acceleration in g force
    float y_g;    // Y acceleration in g force
    float z_g;    // Z acceleration in g force
    float roll;   // Roll angle calculated from accelerometer
    float pitch;  // Pitch angle calculated from accelerometer
};

class RobotKinematics {
private:
    RobotConfig _config;
    JointAngles _currentAngles;
    CartesianPose _currentPose;
    Adafruit_ADXL345_Unified _accel;

public:
    // Constructor
    RobotKinematics(const RobotConfig& config);
    
    // Getters and setters
    void setCurrentJointAngles(const JointAngles& angles);
    JointAngles getCurrentJointAngles() const;
    CartesianPose getCurrentPose() const;
    void setToolOffset(float x, float y, float z);
    
    // Kinematics
    CartesianPose forwardKinematics(const JointAngles& angles);
    bool inverseKinematics(const CartesianPose& targetPose, JointAngles& outAngles);
    bool isPoseReachable(const CartesianPose& pose);
    
    // Helper functions
    float normalizeAngle(float angle);
    
    // Accelerometer
    AccelData getToolAcceleration();
};

#endif // ROBOT_KINEMATICS_H
