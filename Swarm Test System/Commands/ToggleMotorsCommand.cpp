#include "commands/ToggleMotorsCommand.h"
#include <iostream>
#include <frc/Timer.h>
#include <units/length.h>

ToggleMotorsCommand::ToggleMotorsCommand(MotorSubsystem* subsystem) : m_motorSubsystem(subsystem) {
    AddRequirements({subsystem});
}

void ToggleMotorsCommand::Initialize() {
    m_motorSubsystem->ResetEncoder();
    initialSpeed = 0.0;
    currentTime = 0.0;
    rampRate = (goalSpeed - initialSpeed) / duration;

    while (currentTime < duration) {
        currentSpeed = initialSpeed + (rampRate * currentTime);
        m_motorSubsystem->SetMotorSpeed(currentSpeed);
        currentTime += 0.05;
        frc::Wait(0.05_s);
    }
    m_motorSubsystem->SetMotorSpeed(goalSpeed); 

    auto currPos = m_motorSubsystem->GetDistance();
    while (currPos.to<double>() < 3) {
        std::cout << "second loop" << std::endl;
        currPos = m_motorSubsystem->GetDistance();
        std::cout << "position: " << currPos.to<double>() << std::endl;
    }

    m_motorSubsystem->StopMotors();
}

bool ToggleMotorsCommand::IsFinished() {
    return true; // Ends command if canceled
}
