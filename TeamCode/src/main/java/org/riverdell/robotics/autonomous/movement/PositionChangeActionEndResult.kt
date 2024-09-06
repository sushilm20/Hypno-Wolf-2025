package org.riverdell.robotics.autonomous.movement

enum class PositionChangeActionEndResult
{
    Successful, StuckDetected, ExceededTimeout,
    LocalizationFailure, PathAlgorithmSuccessful, ForcefulTermination
}