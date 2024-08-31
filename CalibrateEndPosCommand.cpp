#include "commands/CalibrateEndPosCommand.h"

CalibrateEndPosCommand::CalibrateEndPosCommand(MotorSubsystem* subsystem) : m_motorSubsystem(subsystem) {
    AddRequirements({subsystem});
}

void CalibrateEndPosCommand::Initialize() {
    m_motorSubsystem->CalibrateEndPosition();
}

bool CalibrateEndPosCommand::IsFinished() {
    return true; // Ends immediately after initialization
}