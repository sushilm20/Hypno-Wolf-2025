package org.riverdell.robotics.subsystems.intake.composite

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import java.util.concurrent.CompletableFuture

class CompositeInteraction(private val robot: HypnoticRobot) : AbstractSubsystem() {
    var state = InteractionCompositeState.Rest
    var attemptedState: InteractionCompositeState? = null
    var attemptTime = System.currentTimeMillis()
    var outtakeLevel = OuttakeLevel.Bar1
    var lastOuttakeBegin = System.currentTimeMillis()

    fun outtakeNext(): CompletableFuture<*> {
        if (state != InteractionCompositeState.Deposit) {
            return CompletableFuture.completedFuture(null)
        }

        if (outtakeLevel.next() == null) {
            return CompletableFuture.completedFuture(null)
        }

        outtakeLevel = outtakeLevel.next()!!
        return robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
            .exceptionally { return@exceptionally null }
    }

    fun outtakePrevious(): CompletableFuture<*> {
        if (state != InteractionCompositeState.Deposit) {
            return CompletableFuture.completedFuture(null)
        }

        if (outtakeLevel.previous() == null) {
            return CompletableFuture.completedFuture(null)
        }

        outtakeLevel = outtakeLevel.previous()!!
        return robot.lift.extendToAndStayAt(outtakeLevel.encoderLevel)
            .exceptionally { return@exceptionally null }
    }

    fun outtakeLevel(newLevel: OuttakeLevel): CompletableFuture<*> {
        if (state != InteractionCompositeState.Deposit) {
            return CompletableFuture.completedFuture(null)
        }

        outtakeLevel = newLevel
        return robot.lift.extendToAndStayAt(newLevel.encoderLevel)
            .exceptionally { return@exceptionally null }
    }

    private fun stateMachineRestrict(
        from: InteractionCompositeState, to: InteractionCompositeState,
        ignoreInProgress: Boolean = false,
        supplier: HypnoticRobot.() -> CompletableFuture<Void>
    ): CompletableFuture<Void> {
        if (state != from) {
            return CompletableFuture.completedFuture(null)
        }

        state = if (!ignoreInProgress) {
            InteractionCompositeState.InProgress
        } else {
            to
        }

        attemptedState = to
        attemptTime = System.currentTimeMillis()

        return supplier(robot)
            .whenComplete { _, exception ->
                exception?.printStackTrace()
                if (!ignoreInProgress) {
                    state = to
                }
            }
    }

    override fun doInitialize() {

    }

    override fun start() {
    }

    fun waitForState(state: InteractionCompositeState,
                     timeout: Long = 10000L): Boolean {
        val start = System.currentTimeMillis()
        while (this.state != state)
        {
            if (System.currentTimeMillis() - start > timeout)
            {
                return false
            }
            Thread.sleep(50L)
        }

        return true
    }
}