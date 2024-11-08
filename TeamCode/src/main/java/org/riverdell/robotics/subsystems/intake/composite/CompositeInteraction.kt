package org.riverdell.robotics.subsystems.intake.composite

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import java.util.concurrent.CompletableFuture

class CompositeInteraction(private val robot: HypnoticRobot) : AbstractSubsystem()
{
    var state = InteractionCompositeState.Rest

    fun wallOuttakeFromRest() =
        stateMachineRestrict(
            InteractionCompositeState.Rest,
            InteractionCompositeState.WallIntakeViaOuttake
        ) {
            CompletableFuture.allOf(
                robot.outtake.depositRotation(),
                robot.outtake.openClaw()
            )
        }

    fun wallOuttakeToOuttakeReady() =
        stateMachineRestrict(
            InteractionCompositeState.WallIntakeViaOuttake,
            InteractionCompositeState.OuttakeReady
        ) {
            CompletableFuture.allOf(
                robot.outtake.closeClaw()
            )
        }

    fun outtakeAndRest() =
        stateMachineRestrict(
            InteractionCompositeState.OuttakeReady,
            InteractionCompositeState.Rest
        ) {
            outtake.forceRotation()
                .thenRunAsync {
                    outtake.openClaw()
                    Thread.sleep(150L)
                    outtake.transferRotation().join()
                }
        }

    fun prepareForPickup() =
        stateMachineRestrict(InteractionCompositeState.Rest, InteractionCompositeState.Pickup) {
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
        stateMachineRestrict(InteractionCompositeState.Pickup, InteractionCompositeState.Confirm) {
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
        stateMachineRestrict(InteractionCompositeState.Confirm, InteractionCompositeState.Pickup) {
            CompletableFuture.allOf(
                intakeV4B.v4bSampleGateway(),
                intakeV4B.coaxialIntake()
                    .thenCompose {
                        intake.openIntake()
                    },
                intake.lateralWrist()
            )
        }

    fun confirmAndTransferAndReady() =
        stateMachineRestrict(InteractionCompositeState.Confirm, InteractionCompositeState.OuttakeReady) {
            intakeV4B.v4bTransfer()
                .thenRunAsync {
                    CompletableFuture.allOf(
                        extension.extendToAndStayAt(55),
                        intakeV4B.coaxialRest()
                    ).join()

                    intakeV4B.coaxialTransfer().join()

                    outtake.closeClaw()
                    Thread.sleep(150)

                    intake.openIntake()
                    Thread.sleep(100)

                    outtake.depositRotation()
                    intakeV4B.coaxialRest()
                        .thenRun {
                            intake.closeIntake()
                        }
                        .join()

                    extension.extendToAndStayAt(0).join()
                    intakeV4B.v4bLock()
                }
        }

    private fun stateMachineRestrict(
        from: InteractionCompositeState, to: InteractionCompositeState,
        supplier: HypnoticRobot.() -> CompletableFuture<Void>
    ): CompletableFuture<Void>
    {
        if (state != from)
        {
            return CompletableFuture.completedFuture(null)
        }

        state = InteractionCompositeState.InProgress
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