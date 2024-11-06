package org.riverdell.robotics.subsystems.intake.composite

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import java.util.concurrent.CompletableFuture

class CompositeIntake(private val robot: HypnoticRobot) : AbstractSubsystem()
{
    var state = IntakeCompositeState.Rest
    var current: CompletableFuture<*>? = null
    fun prepareForPickup() =
        stateMachineRestrict(IntakeCompositeState.Rest, IntakeCompositeState.Pickup) {
            intakeV4B.v4bUnlock()
                .thenAcceptAsync {
                    extension.extendToAndStayAt(IntakeConfig.MAX_EXTENSION)
                        .thenAccept {
                            extension.slides.idle()
                        }

                    outtake.transferRotation()
                    outtake.openClaw()

                    CompletableFuture.allOf(
                        intakeV4B.v4bSampleGateway(),
                        intakeV4B.coaxialIntake()
                            .thenCompose {
                                intake.openIntake()
                            },
                        intake.lateralWrist()
                    ).join()
                }
        }

    fun intakeAndConfirm() =
        stateMachineRestrict(IntakeCompositeState.Pickup, IntakeCompositeState.Confirm) {
            intakeV4B.v4bSamplePickup()
                .thenRunAsync {
                    CompletableFuture.allOf(
                        intake.closeIntake(),
                        CompletableFuture.runAsync {
                            Thread.sleep(200L)
                            intake.lateralWrist()
                        },
                        intakeV4B.v4bIntermediate(),
                        intakeV4B.coaxialIntermediate()
                    ).join()
                }
        }

    fun declineAndIntake() =
        stateMachineRestrict(IntakeCompositeState.Confirm, IntakeCompositeState.Pickup) {
            CompletableFuture.allOf(
                intakeV4B.v4bSampleGateway(),
                intakeV4B.coaxialIntake()
                    .thenCompose {
                        intake.openIntake()
                    },
                intake.lateralWrist()
            )
        }

    fun confirmAndTransferAndRest() =
        stateMachineRestrict(IntakeCompositeState.Confirm, IntakeCompositeState.Rest) {
            outtake.openClaw()
            intakeV4B.v4bTransfer()
                .thenRunAsync {
                    CompletableFuture.allOf(
                        extension.extendToAndStayAt(55),
                        intakeV4B.coaxialRest()
                    ).join()

                    intakeV4B.coaxialTransfer().join()

                    outtake.closeClaw()
                    Thread.sleep(250L)

                    intake.openIntake()

                    Thread.sleep(500L)
                    outtake.depositRotation()
                    intakeV4B.coaxialRest()
                        .thenRun {
                            intake.closeIntake()
                        }

                    extension.extendToAndStayAt(0).join()
                    intakeV4B.v4bLock()
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

        state = IntakeCompositeState.InProgress
        return supplier(robot)
            .whenComplete { _, _ ->
                state = to
            }
    }

    override fun doInitialize()
    {

    }

    override fun start()
    {
    }
}