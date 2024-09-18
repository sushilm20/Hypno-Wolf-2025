package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.cubicBezier
import org.riverdell.robotics.autonomous.movement.navigateGVF

@Autonomous(name = "Test | GVF", group = "Test")
class TestGVF : HypnoticAuto({ _ ->
    navigateGVF(cubicBezier(
        Pose(-20.0, 0.0, 0.0),
        Pose(-10.0, 20.0, 0.0),
        Pose(0.0, 0.0, 0.0),
        Pose(20.0, -20.0, 0.0)
    ))
})