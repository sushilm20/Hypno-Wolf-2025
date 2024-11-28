package org.riverdell.robotics.subsystems.intake

enum class IntakeState(val positionRight: Double, val positionLeft: Double)
{
    WideOpen(0.0, 1.0),
    Open(0.1, 0.9),
    Closed(0.38, 0.62),
}