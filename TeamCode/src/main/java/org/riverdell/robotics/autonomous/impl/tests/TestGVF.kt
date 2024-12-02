package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.impl.intothedeep.GroundPickupPosition
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.cubicBezier
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.navigateGVF
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo

@Autonomous(name = "Pose HORS", group = "Test")
class TestGVF : HypnoticAuto({ _ ->
    single("") {
        purePursuitNavigateTo(
            PoseWaypoint(Pose(22.0, 25.0, 45.degrees), 25.0),
            PoseWaypoint(Pose(23.33, 34.33, 90.degrees), 25.0)
        )
    }
})