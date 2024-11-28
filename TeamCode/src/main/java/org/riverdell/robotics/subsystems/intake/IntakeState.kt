package org.riverdell.robotics.subsystems.intake

enum class IntakeState(val positionRight: Double, val positionLeft: Double)
{
    WideOpen(0.18, 0.87),
    Open(0.28, 0.77),
    Closed(0.38, 0.67),
    Lock(0.4, 0.65)
}