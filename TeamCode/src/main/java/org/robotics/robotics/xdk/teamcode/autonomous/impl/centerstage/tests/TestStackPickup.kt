package org.robotics.robotics.xdk.teamcode.autonomous.impl.centerstage.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import io.liftgate.robotics.mono.pipeline.single
import org.robotics.robotics.xdk.teamcode.autonomous.AbstractAutoPipeline
import org.robotics.robotics.xdk.teamcode.autonomous.impl.centerstage.stack5
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile
import org.robotics.robotics.xdk.teamcode.subsystem.claw.ExtendableClaw

@Disabled
@Autonomous(name = "Test | Stack Pickup", group = "Test")
class TestStackPickup : AbstractAutoPipeline(
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