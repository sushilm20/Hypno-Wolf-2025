package org.riverdell.robotics.subsystems.outtake

enum class PivotState(val rightPosition: Double)
{
    Initialize(1.0), Scoring(0.50), Hover(0.75), Pickup(0.1)
}