package org.riverdell.robotics.subsystems.intake

enum class WristState(val position: Double)
{
    Lateral(0.51), Perpendicular(0.8), Dynamic(Double.NaN)
}