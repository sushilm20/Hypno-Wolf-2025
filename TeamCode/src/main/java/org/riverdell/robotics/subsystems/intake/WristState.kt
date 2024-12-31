package org.riverdell.robotics.subsystems.intake

enum class WristState(val position: Double)
{
    Lateral(0.45), Perpendicular(0.9), Dynamic(Double.NaN)
}