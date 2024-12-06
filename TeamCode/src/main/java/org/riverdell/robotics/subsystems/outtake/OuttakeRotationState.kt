package org.riverdell.robotics.subsystems.outtake

enum class OuttakeRotationState(val position: Double)
{
    Transfer(1.0), Deposit(0.65), Force(0.3)
}