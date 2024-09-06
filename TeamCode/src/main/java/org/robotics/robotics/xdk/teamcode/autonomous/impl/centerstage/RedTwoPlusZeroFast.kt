package org.robotics.robotics.xdk.teamcode.autonomous.impl.centerstage

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import io.liftgate.robotics.mono.pipeline.single
import org.robotics.robotics.xdk.teamcode.autonomous.AbstractAutoPipeline
import org.robotics.robotics.xdk.teamcode.autonomous.detection.TapeSide
import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile
import org.robotics.robotics.xdk.teamcode.autonomous.position.degrees
import org.robotics.robotics.xdk.teamcode.autonomous.position.navigateTo
import org.robotics.robotics.xdk.teamcode.autonomous.position.purePursuitNavigateTo
import org.robotics.robotics.xdk.teamcode.autonomous.purepursuit.FieldWaypoint

val boardX = -40.0

@Disabled
@Autonomous(name = "Red Close 2+0 Fast", group = "Test")
class TwoPlusZeroCloseFast : AbstractAutoPipeline(


    AutonomousProfile.RedPlayer2TwoPlusZero,
    blockExecutionGroup = { opMode, tapeSide ->
        spikeMark(opMode, tapeSide)
        /*TapeSide.Middle -> Pose(-40.0, -33.0, (-90).degrees)
                        TapeSide.Left -> Pose(-40.0, -39.0, (-90).degrees)
                        TapeSide.Right -> Pose(-40.0, -27.0, (-90).degrees)
                        */

        single("go to backboad") {
            when (tapeSide) {
                TapeSide.Left -> navigateTo(Pose(boardX, -39.0, -90.degrees))
                TapeSide.Middle -> purePursuitNavigateTo(
                    FieldWaypoint(Pose(-5.0, -23.0, 0.degrees), 10.0),
                    FieldWaypoint(Pose(boardX, -33.0, -90.degrees), 10.0))
                TapeSide.Right -> purePursuitNavigateTo(
                    FieldWaypoint(Pose(1.5, -24.75, -35.degrees), 10.0),
                    FieldWaypoint(Pose(0.0, -15.0, -35.degrees), 10.0),
                    FieldWaypoint(Pose(boardX, -25.0, -90.degrees), 10.0)
                )
            }
            dropPixels(opMode)
        }

        single("park") {
            opMode.elevatorSubsystem.configureElevatorManually(0.0)
            purePursuitNavigateTo(
                when (tapeSide) {
                    TapeSide.Left -> FieldWaypoint(Pose(boardX, -39.0, -90.degrees), 15.0)
                    TapeSide.Middle -> FieldWaypoint(Pose(boardX, -33.0, -90.degrees), 15.0)
                    TapeSide.Right -> FieldWaypoint(Pose(boardX, -25.0, -90.degrees), 15.0)
                },
                when (tapeSide) {
                    TapeSide.Left -> FieldWaypoint(Pose(boardX + 15, -40.0, -90.degrees), 10.0)
                    TapeSide.Middle -> FieldWaypoint(Pose(boardX + 15, -40.0, -90.degrees), 10.0)
                    TapeSide.Right -> FieldWaypoint(Pose(boardX + 15, -40.0, -90.degrees), 10.0)
                },
                FieldWaypoint(Pose(boardX + 10, -50.0, -90.degrees), 15.0),
                FieldWaypoint(Pose(boardX - 6, -56.5, -90.degrees), 15.0)
            )
        }
    }
)