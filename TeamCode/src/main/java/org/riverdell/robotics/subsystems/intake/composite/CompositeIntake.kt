package org.riverdell.robotics.subsystems.intake.composite

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import java.util.concurrent.CompletableFuture

class CompositeIntake(private val robot: HypnoticRobot) : AbstractSubsystem()
{
    var state = IntakeCompositeState.Rest
    fun prepareForPickup() = stateMachineRestrict(IntakeCompositeState.Rest, IntakeCompositeState.Pickup) {
        intakeV4B.v4bUnlock()
            .thenCompose {
                CompletableFuture.allOf(
                    extension.extendToAndStayAt(IntakeConfig.MAX_EXTENSION),
                    intakeV4B.v4bSamplePickup(),
                    intakeV4B.coaxialIntake()
                        .thenCompose {
                            intake.openIntake()
                        },
                    intake.lateralWrist()
                )
            }
    }

    fun cancelPickupAndReturnToRest() = stateMachineRestrict(IntakeCompositeState.Pickup, IntakeCompositeState.Rest) {
        intakeV4B.v4bSamplePickup()
            .thenCompose {
                CompletableFuture.allOf(
                    intake.closeIntake(),
                    intakeV4B.coaxialRest(),
                    intake.lateralWrist(),
                    intakeV4B.v4bLock(),
                    extension.extendToAndStayAt(0)
                )
            }
    }

    private fun stateMachineRestrict(
        from: IntakeCompositeState, to: IntakeCompositeState,
        supplier: HypnoticRobot.() -> CompletableFuture<Void>
    ): CompletableFuture<Void>
    {
        if (state != from)
        {
            return CompletableFuture.completedFuture(null)
        }

        state = to
        return supplier(robot)
    }

    override fun doInitialize()
    {

    }

    override fun start()
    {
    }
}