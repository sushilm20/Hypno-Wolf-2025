package org.riverdell.robotics.subsystems.outtake

enum class OuttakeClawState(val position: Double)
{
    Open(1.0), Closed(0.85)
}