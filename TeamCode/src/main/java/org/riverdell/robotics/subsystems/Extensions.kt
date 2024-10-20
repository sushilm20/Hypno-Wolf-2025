package org.riverdell.robotics.subsystems

import io.liftgate.robotics.mono.konfig.Konfig
import io.liftgate.robotics.mono.states.StateHolder
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.utilities.hardware
import org.riverdell.robotics.utilities.managed.ManagedServo
import org.riverdell.robotics.utilities.motionprofile.MotionProfileConstraints

fun StateHolder.motionProfiledServo(name: String, config: Konfig<MotionProfileConstraints>) = ManagedServo(
    HypnoticRobot.instance.hardware(name),
    this,
    config::get
)