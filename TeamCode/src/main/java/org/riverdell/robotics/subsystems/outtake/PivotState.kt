package org.riverdell.robotics.subsystems.outtake

enum class PivotState(val rightPosition: Double)
{
    Initialize(0.75), PostScore(0.7), Scoring(0.59), Hover(0.28), Pickup(0.23)
}