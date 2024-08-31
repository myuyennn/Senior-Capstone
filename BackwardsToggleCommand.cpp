#include "commands/BackwardsToggleCommand.h"
#include <iostream>

BackwardsToggleCommand::BackwardsToggleCommand(MotorSubsystem* subsystem) : m_motorSubsystem(subsystem) {
    AddRequirements({subsystem});
}

void BackwardsToggleCommand::Initialize() {
    // if (isRunning) {
    //     initialSpeed = goalSpeed;
    //     currentTime = 0.0;
    //     rampRate = (0.0 - initialSpeed) / duration;

    //     while (currentTime < duration) {
    //         currentSpeed = initialSpeed + (rampRate * currentTime);
    //         m_motorSubsystem->SetMotorSpeed(currentSpeed);
    //         currentTime += 0.05;
    //         frc::Wait(0.05_s);
    //     }
    //     m_motorSubsystem->StopMotors();
    // } else {
    //     initialSpeed = 0.0;
    //     currentTime = 0.0;
    //     rampRate = (goalSpeed - initialSpeed) / duration;

    //     while (currentTime < duration) {
    //         currentSpeed = initialSpeed + (rampRate * currentTime);
    //         m_motorSubsystem->SetMotorSpeed(currentSpeed);
    //         currentTime += 0.05;
    //         frc::Wait(0.05_s);
    //     }
    //     m_motorSubsystem->SetMotorSpeed(goalSpeed); 
    // }
    // isRunning = !isRunning;

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

    frc::Wait(0.6_s);

    initialSpeed = goalSpeed;
    currentTime = 0.0;
    rampRate = (0.0 - goalSpeed) / duration;

    while (currentTime < duration) {
        currentSpeed = initialSpeed + (rampRate * currentTime);
        m_motorSubsystem->SetMotorSpeed(currentSpeed);
        currentTime += 0.05;
        frc::Wait(0.05_s);
    }
    m_motorSubsystem->StopMotors();
    m_motorSubsystem->ResetEncoder();
}

bool BackwardsToggleCommand::IsFinished() {
    return true; // Ends command after executing Initialize
}
