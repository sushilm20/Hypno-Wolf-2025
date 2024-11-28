package org.riverdell.robotics.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(
    name = "Solo",
    group = "Drive"
)
class Solo : HypnoticTeleOp(true)