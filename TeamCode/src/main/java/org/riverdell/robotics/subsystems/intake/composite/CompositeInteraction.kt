package org.riverdell.robotics.subsystems.intake.composite

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import java.util.concurrent.CompletableFuture
import kotlin.math.max
import kotlin.math.min

class CompositeInteraction(private val robot: HypnoticRobot) : AbstractSubsystem()
{
    var state = InteractionCompositeState.Rest
    var attemptedState: InteractionCompositeState? = null
    var attemptTime = System.currentTimeMillis()
    var outtakeLevel = OuttakeLevel.Bar1

    fun outtakeNext(): CompletableFuture<*>
    {
        if (state != InteractionCompositeState.Outtaking)
        {
            return CompletableFuture.completedFuture(null)
        }

        if (outtakeLevel.next() == null)
        {
            return CompletableFuture.completedFuture(null)
        }

        outtakeLevel = outtakeLevel.next()!!
        return robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
    }

    fun initialOuttake() = stateMachineRestrict(
        InteractionCompositeState.OuttakeReady,
        InteractionCompositeState.Outtaking
    ) {
        outtakeLevel = OuttakeLevel.Bar1
        CompletableFuture.allOf(
            robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
        )
    }

    fun outtakeCompleteAndRestSimple() = stateMachineRestrict(
        InteractionCompositeState.Outtaking,
        InteractionCompositeState.Rest
    ) {
        CompletableFuture.runAsync {
            outtake.openClaw()
            Thread.sleep(500L)

            CompletableFuture.allOf(
                outtake.transferRotation(),
                robot.lift.extendToAndStayAt(0)
            ).join()
        }
    }

    fun outtakeCompleteAndRest() = stateMachineRestrict(
        InteractionCompositeState.Outtaking,
        InteractionCompositeState.Rest
    ) {

        robot.lift.extendToAndStayAt(max(robot.lift.position() - 50, 0))
            .thenRunAsync {
                outtake.forceRotation()
                Thread.sleep(300L)
                outtake.openClaw()
                CompletableFuture.allOf(
                    outtake.transferRotation(),
                    robot.lift.extendToAndStayAt(0)
                ).join()
            }
    }

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

    fun cancelOuttakeReadyToRest() = stateMachineRestrict(
        InteractionCompositeState.OuttakeReady,
        InteractionCompositeState.Rest
    ) {
        CompletableFuture.allOf(
            robot.outtake.openClaw(),
            robot.outtake.transferRotation()
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
                    Thread.sleep(300L)
                    outtake.openClaw()
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
                        extension.extendToAndStayAt(65),
                        intakeV4B.coaxialRest()
                    ).join()

                    intakeV4B.coaxialTransfer().join()
                    extension.extendToAndStayAt(25).join()

                    outtake.closeClaw()
                    Thread.sleep(150)
                    intake.openIntake()
                    Thread.sleep(100)

                    extension.extendToAndStayAt(55).join()
                    outtake.depositRotation()
                    intakeV4B.coaxialRest().join()
                    intake.closeIntake()
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
        attemptedState = to
        attemptTime = System.currentTimeMillis()

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