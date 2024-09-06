package org.robotics.robotics.xdk.teamcode.autonomous.position

enum class PositionChangeActionEndResult
{
    Successful, StuckDetected, ExceededTimeout,
    LocalizationFailure, PathAlgorithmSuccessful, ForcefulTermination
}