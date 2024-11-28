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
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel

@Autonomous(name = "4+0 Basket & Park", group = "Test")
class PreLoadBasket : HypnoticAuto({ opMode ->
    val startPose = Pose2d(0.0, 0.0, 0.degrees)
    val startPoseHypnotic = Pose(startPose.x, startPose.y, startPose.heading)
    opMode.robot.drivetrain.localizer.poseEstimate = startPose

    var preLoadCompleted = false
    fun ExecutionGroup.depositToHighBasket()
    {
        single("high basket deposit") {
            navigateTo(Pose(19.0, 33.0, 45.degrees))
            if (!preLoadCompleted)
            {
                preLoadCompleted = true
                opMode.robot.intakeComposite
                    .initialOuttakeFromRest(OuttakeLevel.HighBasket)
                    .join()
            } else
            {
                opMode.robot.intakeComposite
                    .initialOuttake(OuttakeLevel.HighBasket)
                    .join()
            }

            navigateTo(Pose(16.0, 35.0, 45.degrees))
            opMode.robot.intakeComposite.outtakeCompleteAndReturnToOuttakeReady()
            Thread.sleep(350L)

            navigateTo(Pose(15.0, 30.0, 90.degrees))
            opMode.robot.intakeComposite.exitOuttakeReadyToRest()
        }
    }

    fun ExecutionGroup.prepareForIntake(
        at: Pose,
        wristState: WristState = WristState.Lateral,
        submersible: Boolean = false,
    )
    {
        /**
         * With the submersible pickups, we want to avoid
         * entering the pickup state until we're perfectly
         * in the pickup position.
         *
         * So, when [submersible] is true, we'll do a consecutive
         * rather than a simultaneous.
         */
        if (!submersible)
        {
            simultaneous("prepare for ground intake") {
                single("navigate to pickup position") {
                    navigateTo(at)
                }

                single("prepare for pickup") {
                    opMode.robot.intakeComposite
                        .prepareForPickup(wristState, wideOpen = true)
                        .join()

                    Thread.sleep(1000L)
                }
            }
        } else
        {
            single("navigate to pickup position") {
                navigateTo(at)
            }

            single("prepare for pickup") {
                opMode.robot.intakeComposite
                    .prepareForPickup(wristState)
                    .join()
            }
        }
    }

    fun ExecutionGroup.confirmIntakeAndTransfer()
    {
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
        GroundPickupPosition(pose = Pose(20.0, 33.0, 90.degrees)),
        GroundPickupPosition(pose = Pose(20.0, 50.0, 90.degrees)),
        GroundPickupPosition(pose = Pose(47.0, 24.0, 180.degrees), wristState = WristState.Perpendicular),
        // TODO: add 2nd, and 3rd ground sample poses & wrist states
    )

    // ground pickup positions
    pickupPositions.forEach {
        prepareForIntake(
            at = it.pose,
            wristState = it.wristState
        )
        confirmIntakeAndTransfer()
        depositToHighBasket()
    }

    single("park near submersible") {
        navigateTo(startPoseHypnotic)
    }
})