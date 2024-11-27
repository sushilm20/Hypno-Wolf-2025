package org.riverdell.robotics.subsystems.intake.composite

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.intake.WristState
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

    fun outtakeLevel(newLevel: OuttakeLevel): CompletableFuture<*>
    {
        if (state != InteractionCompositeState.Outtaking)
        {
            return CompletableFuture.completedFuture(null)
        }

        outtakeLevel = newLevel
        return robot.lift.extendToAndStayAt(newLevel.encoderLevel)
    }

    fun initialOuttakeFromRest(preferredLevel: OuttakeLevel = OuttakeLevel.Bar1) = stateMachineRestrict(
        InteractionCompositeState.Rest,
        InteractionCompositeState.Outtaking
    ) {
        outtakeLevel = preferredLevel
        robot.outtake.depositRotation()

        CompletableFuture.allOf(
            robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
        )
    }

    fun initialOuttake(preferredLevel: OuttakeLevel = OuttakeLevel.Bar1) = stateMachineRestrict(
        InteractionCompositeState.OuttakeReady,
        InteractionCompositeState.Outtaking
    ) {
        outtakeLevel = preferredLevel
        CompletableFuture.allOf(
            robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
        )
    }

    fun outtakeCompleteAndRestSimple() = stateMachineRestrict(
        InteractionCompositeState.Outtaking,
        InteractionCompositeState.OuttakeReady
    ) {
        CompletableFuture.runAsync {
            outtake.openClaw()
        }
    }

    fun outtakeCompleteAndRest() = stateMachineRestrict(
        InteractionCompositeState.Outtaking,
        InteractionCompositeState.Rest
    ) {
        robot.lift.extendToAndStayAt(0)

        outtake.forceRotation()
        outtake.openClaw()

        CompletableFuture.allOf(
            outtake.transferRotation()
        )
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
            robot.outtake.transferRotation(),
            robot.lift.extendToAndStayAt(0)
        )
    }

    fun inToOut() = stateMachineRestrict(
        InteractionCompositeState.Rest,
        InteractionCompositeState.OuttakeReady
    ) {
        CompletableFuture.allOf(
            robot.outtake.depositRotation()
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

    fun prepareForPickup(wristState: WristState = WristState.Lateral) =
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
                        intake.setWrist(wristState)
                    ).join()
                }
        }

    fun intakeAndConfirm() =
        stateMachineRestrict(InteractionCompositeState.Pickup, InteractionCompositeState.Confirm) {
            CompletableFuture.allOf(
                CompletableFuture.runAsync {
                    Thread.sleep(500L)
                    intake.closeIntake()
                },
                intakeV4B.v4bSamplePickup()
            ).thenRunAsync {
                CompletableFuture.allOf(
                    intakeV4B.v4bIntermediate(),
                    intakeV4B.coaxialIntermediate()
                ).join()

                intake.wristState = WristState.Lateral
                intake.wrist.unwrapServo().position = WristState.Lateral.position
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
        stateMachineRestrict(
            InteractionCompositeState.Confirm,
            InteractionCompositeState.OuttakeReady
        ) {
            intake.lockIntake()
            intakeV4B.v4bTransfer()
                .thenRunAsync {
                    CompletableFuture.allOf(
                        extension.extendToAndStayAt(135),
                        intakeV4B.coaxialRest()
                    ).join()

                    intakeV4B.coaxialTransfer().join()
                    extension.extendToAndStayAt(80).join()

                    outtake.closeClaw()
                    Thread.sleep(150)
                    intake.openIntake()
                    Thread.sleep(100)

                    extension.extendToAndStayAt(150).join()
                    outtake.depositRotation()
                    intakeV4B.coaxialRest()

                    Thread.sleep(250L)
                    extension.extendToAndStayAt(0).join()

                    intakeV4B.v4bLock().join()
                    intake.closeIntake()
                    extension.slides.idle()
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