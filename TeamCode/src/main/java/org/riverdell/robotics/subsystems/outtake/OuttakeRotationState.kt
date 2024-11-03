package org.riverdell.robotics.subsystems.outtake

enum class OuttakeRotationState(val position: Double)
{
    Transfer(0.0), Deposit(1.0)
}