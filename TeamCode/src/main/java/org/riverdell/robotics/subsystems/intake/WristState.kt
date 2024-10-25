package org.riverdell.robotics.subsystems.intake

enum class WristState(val position: Double)
{
    Lateral(0.5), Perpendicular(1.0), Dynamic(Double.NaN)
}