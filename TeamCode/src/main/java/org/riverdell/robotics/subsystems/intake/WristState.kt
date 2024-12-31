package org.riverdell.robotics.subsystems.intake

enum class WristState(val position: Double)
{
    Lateral(0.49), Perpendicular(0.89), Dynamic(Double.NaN)
}