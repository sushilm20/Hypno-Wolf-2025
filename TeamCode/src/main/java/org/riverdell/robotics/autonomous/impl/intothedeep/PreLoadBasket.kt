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
import org.riverdell.robotics.subsystems.intake.composite.InteractionCompositeState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel

@Autonomous(name = "4+0 Basket", group = "Test")
class PreLoadBasket : HypnoticAuto({ opMode ->
    val startPose = Pose2d(0.0, 0.0, 0.degrees)

    val depositHighBucket = Pose(12.36, 23.84, (41.65).degrees)

    val parkSubmersible = listOf(
        FieldWaypoint(depositHighBucket, 15.0),
        FieldWaypoint(Pose(70.29, 22.78, (-177.78).degrees), 25.0),
        FieldWaypoint(
            Pose(70.12, -30.32, (178.92).degrees), 15.0
        ),
        FieldWaypoint(
            Pose(70.12, -30.32, (178.92).degrees), 15.0
        ),
    )

    opMode.robot.drivetrain.localizer.poseEstimate = startPose

    var preLoadCompleted = false
    fun ExecutionGroup.depositToHighBasket(initial: Boolean = false, alternatePose: Pose? = null) {
        single("high basket deposit") {
            if (!preLoadCompleted) {
                preLoadCompleted = true
                opMode.robot.intakeComposite
                    .initialOuttakeFromRest(OuttakeLevel.HighBasket)
                    .join()
            } else {
                opMode.robot.intakeComposite
                    .initialOuttake(OuttakeLevel.HighBasket)
            }

            if (initial)
            {
                navigateTo(depositHighBucket)
            } else
            {
                if (alternatePose != null)
                {
                    navigateTo(alternatePose)
                } else
                {
                    navigateTo(depositHighBucket)
                }
            }

            opMode.robot.intakeComposite.outtakeCompleteAndRest().join()
        }
    }

    fun ExecutionGroup.prepareForIntake(
        position: GroundPickupPosition
    ) {

        single("navigate to pickup position") {
            opMode.robot.intakeComposite.prepareForPickup(
                position.wristState,
                // needs to go closer into the wall
                doNotUseAutoMode = position.extraPoseBack != null,
                wideOpen = true
            )

            if (position.purePursuitPoints != null) {
                purePursuitNavigateTo(*position.purePursuitPoints.toTypedArray()) {
                    withAutomaticDeath(5000.0)
                }
                navigateTo(position.pose) {
                    withCustomMaxRotationalSpeed(0.4)
                    withCustomMaxTranslationalSpeed(0.4)
                }
            } else {
                navigateTo(position.pose)
            }

            if (opMode.robot.intakeComposite.state != InteractionCompositeState.Pickup) {
                val currentPose = opMode.robot.drivetrain.localizer.pose
                if (position.extraPoseBack != null) {
                    navigateTo(position.extraPoseBack)
                }

                opMode.robot.intakeV4B.v4bSampleGateway().join()
                opMode.robot.intakeComposite.state = InteractionCompositeState.Pickup

                if (position.extraPoseBack != null) {
                    navigateTo(currentPose)
                }
            }

            Thread.sleep(250L)
        }
    }

    fun ExecutionGroup.confirmIntakeAndTransfer(pose: Pose?) {
        single("confirm intake") {
            opMode.robot.intakeComposite
                .intakeAndConfirm {
                    if (pose != null) {
                        navigateTo(pose)
                    }
                }
                .join()

            opMode.robot.intakeComposite
                .confirmAndTransferAndReady()
                .join()
        }
    }

    // preload
    depositToHighBasket(initial = true)

    val pickupPositions = listOf(
        GroundPickupPosition(pose = Pose(15.07, 19.50, (87.93).degrees)),
        GroundPickupPosition(pose = Pose(13.88, 33.63, (89.82).degrees)),
        GroundPickupPosition(
            pose = Pose(44.73, 10.1, (178.44).degrees),
            extraPoseBack = Pose(44.67, 3.29, (179.40).degrees),
            purePursuitPoints = listOf(
                FieldWaypoint(depositHighBucket, 15.0),
                FieldWaypoint(Pose(44.67, 3.29, (179.40).degrees), 15.0),
                FieldWaypoint(Pose(44.73, 10.1, (178.44).degrees), 15.0),
                FieldWaypoint(Pose(44.73, 10.1, (178.44).degrees), 15.0),
            ),
            alternateHighBasketDriftComp = Pose(11.37, 21.69, (44.66).degrees),
            wristState = WristState.Perpendicular
        ),
    )

    // ground pickup positions
    pickupPositions.forEach {
        prepareForIntake(it)
        confirmIntakeAndTransfer(it.extraPoseBack)
        depositToHighBasket(alternatePose = it.alternateHighBasketDriftComp)
    }

    single("park near submersible") {
        opMode.robot.intakeComposite
            .initialOuttakeFromRest(OuttakeLevel.Bar1)

        purePursuitNavigateTo(*parkSubmersible.toTypedArray()) {
            withAutomaticDeath(9000.0)
            withCustomMaxTranslationalSpeed(0.3)
            withCustomMaxRotationalSpeed(0.3)
        }
    }
})