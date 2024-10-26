package org.riverdell.robotics.subsystems.intake

enum class CoaxialState(val position: Double)
{
    Intake(1.0), Transfer(0.0), Intermediate(0.5)
}