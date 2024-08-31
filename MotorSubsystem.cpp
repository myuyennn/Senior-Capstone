#include "subsystems/MotorSubsystem.h"

#include <iostream>

MotorSubsystem::MotorSubsystem() 
    : masterMotor(1, rev::CANSparkMax::MotorType::kBrushless),
      masterEncoder(masterMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      followerEncoder2(followerMotor2.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      followerEncoder3(followerMotor3.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      followerEncoder4(followerMotor4.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      followerEncoder5(followerMotor5.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      followerEncoder6(followerMotor6.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      followerEncoder7(followerMotor7.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      followerEncoder8(followerMotor8.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      followerEncoder9(followerMotor9.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      followerEncoder10(followerMotor10.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      followerEncoder11(followerMotor11.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      followerEncoder12(followerMotor12.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)) {

    masterEncoder.SetPositionConversionFactor(42.0);
    followerEncoder2.SetPositionConversionFactor(42.0);
    followerEncoder3.SetPositionConversionFactor(42.0);
    followerEncoder4.SetPositionConversionFactor(42.0);
    followerEncoder5.SetPositionConversionFactor(42.0);
    followerEncoder6.SetPositionConversionFactor(42.0);
    followerEncoder7.SetPositionConversionFactor(42.0);
    followerEncoder8.SetPositionConversionFactor(42.0);
    followerEncoder9.SetPositionConversionFactor(42.0);
    followerEncoder10.SetPositionConversionFactor(42.0);
    followerEncoder11.SetPositionConversionFactor(42.0);
    followerEncoder12.SetPositionConversionFactor(42.0);

    ResetEncoder();

    ConfigureFollowers();
}

void MotorSubsystem::SetMotorSpeed(double speed) {
    masterMotor.Set(speed);
}

void MotorSubsystem::StopMotors() {
    masterMotor.Set(0);
}

double MotorSubsystem::GetRPM() const {
    return masterEncoder.GetVelocity();
}

void MotorSubsystem::ConfigureFollowers() {
    followerMotor2.Follow(masterMotor, false); // invert = false -> follow direction of leader
    followerMotor3.Follow(masterMotor, false);
    followerMotor4.Follow(masterMotor, false);
    followerMotor5.Follow(masterMotor, false);
    followerMotor6.Follow(masterMotor, false);
    followerMotor7.Follow(masterMotor, true); // invert = true -> follow opposite direction of leader
    followerMotor8.Follow(masterMotor, true);
    followerMotor9.Follow(masterMotor, true);
    followerMotor10.Follow(masterMotor, true);
    followerMotor11.Follow(masterMotor, true);
    followerMotor12.Follow(masterMotor, true); 
}
    
double MotorSubsystem::GetCurrentSpeed() const {
    return currSpeed;  // Return the last set speed
}

void MotorSubsystem::ResetEncoder() {
    masterEncoder.SetPosition(0.0);
    followerEncoder2.SetPosition(0.0);
    followerEncoder3.SetPosition(0.0);
    followerEncoder4.SetPosition(0.0);
    followerEncoder5.SetPosition(0.0);
    followerEncoder6.SetPosition(0.0);
    followerEncoder7.SetPosition(0.0);
    followerEncoder8.SetPosition(0.0);
    followerEncoder9.SetPosition(0.0);
    followerEncoder10.SetPosition(0.0);
    followerEncoder11.SetPosition(0.0);
    followerEncoder12.SetPosition(0.0);
}

units::meter_t MotorSubsystem::GetDistance() const {
    double scaledDistance = (masterEncoder.GetPosition()) / countsPerMeter;
    return units::meter_t{scaledDistance};
}

units::meters_per_second_t MotorSubsystem::GetVelocity() const {
    // double rpm = masterEncoder.GetVelocity();
    // double countsPerSecond = (masterEncoder.GetVelocity()) / 60.0; // RPM -> RPS
    // double metersPerSecond = countsPerSecond / countsPerMeter; // Convert to meters per second
    // std::cout << "rpm: " << rpm << std::endl;
    // std::cout << "counts/sec: " << countsPerSecond << std::endl;
    // std::cout << "Velocity (m/s): " << metersPerSecond << std::endl;
    double metersPerSecond = masterEncoder.GetVelocity() * 0.2921;
    std::cout << "Velocity (m/s): " << metersPerSecond << std::endl;
    return units::meters_per_second_t{metersPerSecond};
}

void MotorSubsystem::CalibrateStartPosition() {
    // START CARRIAGE AT START OF LAUNCHER
    ResetEncoder();
    frc::Wait(0.5_s);
    startPos = masterEncoder.GetPosition();
    double rawEncoderPos = masterEncoder.GetPosition();
    double positionCF = masterEncoder.GetPositionConversionFactor();
    double followerEncoder2Pos = followerEncoder2.GetPosition();
    units::meter_t distanceInMeters = GetDistance();
    std::cout << "Raw Encoder Start Position: " << rawEncoderPos << std::endl;
    std::cout << "Follower Motor Raw Encoder Start Pos: " << followerEncoder2Pos << std::endl;
    std::cout << "Distance in Meters: " << distanceInMeters.to<double>() << std::endl;
    //std::cout << "Position Conversion Factor: " << positionCF << std::endl;
    printPosition(startPos * positionCF);
    startPositionCalibrated = true;
}

void MotorSubsystem::CalibrateEndPosition() {
    frc::Wait(1.0_s);
    endPos = masterEncoder.GetPosition();
    double rawEncoderPos = masterEncoder.GetPosition();
    double positionCF = masterEncoder.GetPositionConversionFactor();
    double followerEncoder2Pos = followerEncoder2.GetPosition();
    units::meter_t distanceInMeters = GetDistance();
    std::cout << "Raw Encoder End Position: " << rawEncoderPos << std::endl;
    std::cout << "Follower Motor Raw Encoder End Pos: " << followerEncoder2Pos << std::endl;
    std::cout << "Distance in Meters: " << distanceInMeters.to<double>() << std::endl;
    //std::cout << "Position Conversion Factor: " << positionCF << std::endl;
    // std::cout << "Calculated End Position: " << endPos.to<double>() << " meters" << std::endl;
    printPosition(endPos * positionCF);
    endPositionCalibrated = true;
}

double MotorSubsystem::GetStartPosition() const {
    return startPos;
}

double MotorSubsystem::GetEndPosition() const {
    return endPos;
}

bool MotorSubsystem::isCalibrated() {
    return startPositionCalibrated & endPositionCalibrated; 
}

void MotorSubsystem::printPosition(double position) {
    std::cout << "Encoder Position: " << position << std::endl;
}

void MotorSubsystem::Periodic() {
    motorCurrent = masterMotor.GetOutputCurrent();
    motorVoltage = masterMotor.GetBusVoltage();
    if (counter++ % 10000 == 0) {
        std::cout << "Motor Current: " << motorCurrent << " A" << std::endl;
        std::cout << "Motor Voltage: " << motorVoltage << " V" << std::endl;
        counter = 1;
    }
}