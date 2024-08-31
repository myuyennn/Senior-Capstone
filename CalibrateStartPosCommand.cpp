#include "commands/CalibrateStartPosCommand.h"

CalibrateStartPosCommand::CalibrateStartPosCommand(MotorSubsystem* subsystem) : m_motorSubsystem(subsystem) {
    AddRequirements({subsystem});
}

void CalibrateStartPosCommand::Initialize() {
    m_motorSubsystem->CalibrateStartPosition();
}

bool CalibrateStartPosCommand::IsFinished() {
    return true; // Ends immediately after initialization
}