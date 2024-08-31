#include "commands/EmergencyStopCommand.h"

EmergencyStopCommand::EmergencyStopCommand(MotorSubsystem* subsystem) : m_motorSubsystem(subsystem) {
    AddRequirements({subsystem});
}

void EmergencyStopCommand::Initialize() {
    m_motorSubsystem->StopMotors();
}

bool EmergencyStopCommand::IsFinished() {
    return true; // Ends immediately after initialization
}