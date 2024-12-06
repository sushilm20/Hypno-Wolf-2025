package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo
import org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint

@Autonomous(name = "Pose HORS", group = "Test")
class TestGVF : HypnoticAuto({ _ ->
    single("") {
        purePursuitNavigateTo(
            FieldWaypoint(Pose(), 10.0),
            FieldWaypoint(Pose(64.0, 35.0, 1.6), 20.0),
            FieldWaypoint(Pose(156.0, 29.0, 0.7), 20.0),
            FieldWaypoint(Pose(170.0, -110.0, -0.8), 20.0),
            FieldWaypoint(Pose(23.0, -122.0, -2.49), 20.0),
            FieldWaypoint(Pose(), 10.0)
        )
    }
})