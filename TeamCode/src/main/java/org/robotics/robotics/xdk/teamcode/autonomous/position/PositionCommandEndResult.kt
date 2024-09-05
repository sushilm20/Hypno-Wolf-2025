package org.robotics.robotics.xdk.teamcode.autonomous.position

enum class PositionCommandEndResult
{
    Successful, StuckDetected, ExceededTimeout,
    LocalizationFailure, PathAlgorithmSuccessful, ForcefulTermination
}