package org.riverdell.robotics.autonomous.impl.intothedeep

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.ExecutionGroup
import io.liftgate.robotics.mono.pipeline.simultaneous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel

@Autonomous(name = "4+0 Basket & hors", group = "Test")
class PreLoadBasket : HypnoticAuto({ opMode ->
    val startPose = Pose2d(0.0, 0.0, 0.degrees)
    val startPoseHypnotic = Pose(startPose.x, startPose.y, startPose.heading)
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

            navigateTo(Pose(12.5, 23.5, 39.degrees))
            opMode.robot.intakeComposite.outtakeCompleteAndRest().join()
        }
    }

    fun ExecutionGroup.prepareForIntake(
        position: GroundPickupPosition,
        submersible: Boolean = false,
    ) {
        /**
         * With the submersible pickups, we want to avoid
         * entering the pickup state until we're perfectly
         * in the pickup position.
         *
         * So, when [submersible] is true, we'll do a consecutive
         * rather than a simultaneous.
         */
        if (!submersible) {
            single("navigate to pickup position") {
                opMode.robot.intakeComposite.prepareForPickup(position.wristState, wideOpen = true)
                if (position.purePursuitPoints != null)
                {
                    purePursuitNavigateTo(*position.purePursuitPoints.toTypedArray())
                } else
                {
                    navigateTo(position.pose)
                }

                Thread.sleep(500L)
            }
        } else {
            // SUMBERSIBLE! DO NOT TOUCH
            simultaneous("thing") {
                single("navigate to pickup position") {
                    navigateTo(position.pose)
                }

                single("prepare for pickup") {
                    opMode.robot.intakeComposite
                        .prepareForPickup(position.wristState)
                        .join()
                }
            }
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
        GroundPickupPosition(pose = Pose(22.00, 19.0, 90.degrees)),
        GroundPickupPosition(pose = Pose(22.00, 36.0, 90.degrees)),
        /*GroundPickupPosition(
            pose = Pose(44.98, 33.8, 180.degrees),
            wristState = WristState.Perpendicular
        ),*/
    )

    // ground pickup positions
    pickupPositions.forEach {
        prepareForIntake(it)
        confirmIntakeAndTransfer()
        depositToHighBasket()
    }

    single("park near submersible") {
        navigateTo(startPoseHypnotic)
    }
})