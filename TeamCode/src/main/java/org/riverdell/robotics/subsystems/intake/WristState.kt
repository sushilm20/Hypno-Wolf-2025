package org.riverdell.robotics.subsystems.intake

enum class WristState(val position: Double)
{
    Lateral(0.5), Perpendicular(0.8), Dynamic(Double.NaN)
}