package org.riverdell.robotics.autonomous.impl.intothedeep

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.ExecutionGroup
import io.liftgate.robotics.mono.pipeline.simultaneous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Point
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo
import org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel

@Autonomous(name = "4+0 Basket & hors", group = "Test")
class PreLoadBasket : HypnoticAuto({ opMode ->
    val startPose = Pose2d(0.0, 0.0, 0.degrees)
    val depositHighBucket = Pose(16.61, 26.33, 47.degrees)
    val parkSubmersible = listOf(
        FieldWaypoint(depositHighBucket, 15.0),
        FieldWaypoint(Pose(55.01, -5.38, (-130).degrees), 15.0),
        FieldWaypoint(Pose(72.22, -25.44, (-180).degrees), 15.0),
        FieldWaypoint(Pose(72.22, -25.44, (-180).degrees), 15.0),
    )
    opMode.robot.drivetrain.localizer.poseEstimate = startPose

    var preLoadCompleted = false
    fun ExecutionGroup.depositToHighBasket() {
        single("high basket deposit") {
            if (!preLoadCompleted) {
                preLoadCompleted = true
                opMode.robot.intakeComposite
                    .initialOuttakeFromRest(OuttakeLevel.HighBasket)
            } else {
                opMode.robot.intakeComposite
                    .initialOuttake(OuttakeLevel.HighBasket)
            }

            navigateTo(depositHighBucket)
            opMode.robot.intakeComposite.outtakeCompleteAndRest().join()
        }
    }

    fun ExecutionGroup.prepareForIntake(
        position: GroundPickupPosition,
        submersible: Boolean = false,
    ) {

        single("navigate to pickup position") {
            opMode.robot.intakeComposite.prepareForPickup(position.wristState, wideOpen = true)
            if (position.purePursuitPoints != null)
            {
                purePursuitNavigateTo(*position.purePursuitPoints.toTypedArray()) {
                    withAutomaticDeath(5000.0)
                }
                navigateTo(position.pose)
            } else
            {
                navigateTo(position.pose)
            }

            Thread.sleep(500L)
        }
    }

    fun ExecutionGroup.confirmIntakeAndTransfer() {
        single("confirm intake") {
            opMode.robot.intakeComposite
                .intakeAndConfirm()
                .join()

            opMode.robot.intakeComposite
                .confirmAndTransferAndReady()
                .join()
        }
    }

    // preload
    depositToHighBasket()

    val pickupPositions = listOf(
        GroundPickupPosition(pose = Pose(23.38, 19.08, 90.degrees)),
        GroundPickupPosition(pose = Pose(22.71, 34.31, 90.degrees)),
        GroundPickupPosition(
            pose = Pose(44.16, 14.92, 180.degrees),
            purePursuitPoints = listOf(
                FieldWaypoint(depositHighBucket, 15.0),
                FieldWaypoint(Pose(44.35, 4.53, 180.degrees), 15.0),
                FieldWaypoint(Pose(44.16, 14.92, 180.degrees), 15.0),
            ),
            wristState = WristState.Perpendicular
        ),
    )

    // ground pickup positions
    pickupPositions.forEach {
        prepareForIntake(it)
        confirmIntakeAndTransfer()
        depositToHighBasket()
    }

    single("park near submersible") {
        opMode.robot.intakeComposite
            .initialOuttakeFromRest(OuttakeLevel.Bar1)

        purePursuitNavigateTo(*parkSubmersible.toTypedArray()) {
            withAutomaticDeath(5000.0)
        }
    }
})