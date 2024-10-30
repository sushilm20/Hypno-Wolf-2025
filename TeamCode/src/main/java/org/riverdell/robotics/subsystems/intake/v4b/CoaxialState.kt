package org.riverdell.robotics.subsystems.intake.v4b

enum class CoaxialState(val position: Double)
{
    Intake(1.0), Transfer(0.0), Rest(0.3), Intermediate(0.5)
}