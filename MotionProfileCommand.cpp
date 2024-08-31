#include "commands/MotionProfileCommand.h"
#include <frc/Timer.h>

#include <units/acceleration.h>
#include <units/velocity.h>

MotionProfileCommand::MotionProfileCommand(MotorSubsystem* subsystem, const MotionProfile& profile)
    : CommandHelper(
          frc::ProfiledPIDController<units::meters>(
              kP, 
              kI, 
              kD,
              {kMaxSpeedMetersPerSecond, kMaxAccelMetersPerSecondSquared},
              33_ms),
          [subsystem] { return subsystem->GetDistance(); },
          [profile] { return frc::TrapezoidProfile<units::meters>::State{profile[0].position, profile[0].velocity}; },
          [subsystem](double output, frc::TrapezoidProfile<units::meters>::State setpoint) {
              subsystem->SetMotorSpeed(output);
          }),
      m_motorSubsystem(subsystem),  
      m_profile(profile) {
  AddRequirements({subsystem});
  GetController().SetTolerance(0.05_m, 0.05_mps);
}

void MotionProfileCommand::Initialize() {
  m_motorSubsystem->ResetEncoder();
  if (!m_motorSubsystem->isCalibrated()) {
    std::cout << "Subsystem not calibrated. Cancelling command." << std::endl;
    Cancel(); // Cancel command if subsystem is not calibrated
    return;
  }
  index = 0;
  std::cout << "Initializing motion profile command." << std::endl;
}

void MotionProfileCommand::Execute() {
  std::cout << "Starting Motion Profile." << std::endl;
  if (index < m_profile.size()) {
    auto& point = m_profile[index];
    GetController().SetGoal({point.position, point.velocity});
  }

  auto constraints = GetController().GetConstraints();
  auto currPos = m_motorSubsystem->GetDistance();
  auto goal = GetController().GetGoal().position;
  auto velgoal = GetController().GetGoal().velocity;

  auto feedbackOutput = GetController().Calculate(currPos, goal, constraints);

  m_motorSubsystem->GetVelocity();
  
  m_motorSubsystem->SetMotorSpeed(feedbackOutput);

  std::cout << "Current Goal - Position: " << goal.to<double>() << " meters, Velocity: " << velgoal.to<double>() << " meters/second" << std::endl;

  std::cout << "motor output: " << feedbackOutput << std::endl;

  std::cout << "index: " << index << std::endl;

  ++index;
}

// Returns true when the command should end.
bool MotionProfileCommand::IsFinished() {
  auto pos = m_motorSubsystem->GetDistance();
  std::cout << "position: " << pos.to<double>() << std::endl;
  return index >= m_profile.size();
}
