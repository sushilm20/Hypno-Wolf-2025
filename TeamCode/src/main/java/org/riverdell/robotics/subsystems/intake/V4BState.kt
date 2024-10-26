package org.riverdell.robotics.subsystems.intake

enum class V4BState(val position: Double)
{
    Intake(1.0), Transfer(0.2), Lock(0.0), Intermediate(0.5)
}