package org.riverdell.robotics.subsystems.outtake

enum class OuttakeCoaxialState(val position: Double)
{
    Transfer(0.0), Deposit(0.5), OutsideIntake(0.5)
}