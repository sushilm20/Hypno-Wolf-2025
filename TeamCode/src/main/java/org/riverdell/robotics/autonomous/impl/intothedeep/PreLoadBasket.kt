package org.riverdell.robotics.autonomous.impl.intothedeep

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
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
    fun depositToHighBasket()
    {
        single("high basket deposit") {
            navigateTo(Pose(-83.0, 28.0, 128.degrees))
            opMode.robot.intakeComposite
                .initialOuttakeFromRest(OuttakeLevel.HighBasket)
                .join()

            navigateTo(Pose(-83.0, 20.0, 128.degrees))
            opMode.robot.intakeComposite.outtakeCompleteAndRestSimple()
            Thread.sleep(350L)

            navigateTo(Pose(-75.0, 27.0, 128.degrees))
            opMode.robot.intakeComposite.cancelOuttakeReadyToRest().join()
        }
    }

    fun prepareForIntake(
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
                        .prepareForPickup(wristState)
                        .join()
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

    fun confirmIntakeAndTransfer()
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
        GroundPickupPosition(pose = Pose(-82.2, 28.6, 180.degrees)),
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
        // TODO: change submersible park pose
        navigateTo(Pose(0.0, 0.0, 0.0))
    }
})