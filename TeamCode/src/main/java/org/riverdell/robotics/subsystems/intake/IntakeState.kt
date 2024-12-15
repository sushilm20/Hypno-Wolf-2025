package org.riverdell.robotics.subsystems.intake

enum class IntakeState(val positionRight: Double, val positionLeft: Double)
{
    WideOpen(1.0, 0.0),
    Open(0.8, 0.2),
    Closed(0.5, 0.5),
}