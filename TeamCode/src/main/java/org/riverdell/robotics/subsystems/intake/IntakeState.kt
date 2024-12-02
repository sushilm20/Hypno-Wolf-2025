package org.riverdell.robotics.subsystems.intake

enum class IntakeState(val positionRight: Double, val positionLeft: Double)
{
    WideOpen(0.0, 1.0),
    Open(0.2, 0.8),
    Closed(0.48, 0.52),
}