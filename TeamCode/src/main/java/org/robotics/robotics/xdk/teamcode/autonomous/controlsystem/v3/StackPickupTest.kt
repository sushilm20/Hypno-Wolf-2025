package org.robotics.robotics.xdk.teamcode.autonomous.controlsystem.v3

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.robotics.robotics.xdk.teamcode.autonomous.AbstractAutoPipeline
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile
import org.robotics.robotics.xdk.teamcode.subsystem.claw.ExtendableClaw

@Autonomous(name = "Test | Stack Pickup", group = "Test")
class StackPickupTest : AbstractAutoPipeline(
    AutonomousProfile.StackPickupTest,
    blockExecutionGroup = {opMode, tapeSide ->
        single("") {
            opMode.clawSubsystem.updateClawState(
                ExtendableClaw.ClawStateUpdate.Right,
                ExtendableClaw.ClawState.MosaicFix,
            )
            opMode.clawSubsystem.toggleExtender(ExtendableClaw.ExtenderState.Deposit, force = true)
            opMode.elevatorSubsystem.configureElevatorManually(stack5)
            Thread.sleep(10000L)
        }
    }
)