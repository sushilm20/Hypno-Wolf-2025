package org.robotics.robotics.xdk.teamcode.autonomous.impl.centerstage

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import io.liftgate.robotics.mono.pipeline.single
import org.robotics.robotics.xdk.teamcode.autonomous.AbstractAutoPipeline
import org.robotics.robotics.xdk.teamcode.autonomous.detection.TapeSide
import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile
import org.robotics.robotics.xdk.teamcode.autonomous.position.degrees
import org.robotics.robotics.xdk.teamcode.autonomous.position.navigateTo
import org.robotics.robotics.xdk.teamcode.autonomous.position.purePursuitNavigateTo
import org.robotics.robotics.xdk.teamcode.autonomous.purepursuit.ActionWaypoint
import org.robotics.robotics.xdk.teamcode.autonomous.purepursuit.FieldWaypoint
import org.robotics.robotics.xdk.teamcode.subsystem.claw.ExtendableClaw

@Disabled
@Autonomous(name = "Red 2+1", group = "Test")
class RedTwoPlusOne : AbstractAutoPipeline(

    AutonomousProfile.RedPlayer2TwoPlusZero,
    blockExecutionGroup = { opMode, tapeSide ->
        fun moveStackToBackBoard() {
            single("go to backboard from stack") {
                purePursuitNavigateTo(
                    FieldWaypoint(
                        farStackPickup,
                        20.0
                    ),
                    FieldWaypoint(
                        Pose(-5.0, -56.0, (90).degrees),
                        25.0
                    ),
                    FieldWaypoint(
                        Pose(-63.0, -56.0, (90).degrees),
                        25.0
                    ),
                    ActionWaypoint {
                        opMode.elevatorSubsystem.configureElevatorManually(0.0)
                    },
                    FieldWaypoint(
                        Pose(-72.0, -42.0, (180).degrees),
                        25.0
                    ),
                    ActionWaypoint {
                        opMode.clawSubsystem.toggleExtender(
                            ExtendableClaw.ExtenderState.Deposit,
                            force = true
                        )
                    },
                    *when (tapeSide)
                    { //backboard
                        TapeSide.Left -> arrayOf(
                            FieldWaypoint(
                                Pose(-86.85, -34.5, (-90.0).degrees),
                                5.0
                            )
                        )

                        TapeSide.Middle -> arrayOf(
                            FieldWaypoint(
                                Pose(-85.85, -28.0, (-90.0).degrees),
                                10.0
                            )
                        )

                        TapeSide.Right -> arrayOf(
                            FieldWaypoint(
                                Pose(-85.85, -22.5, (-90.0).degrees),
                                20.0
                            )
                        )
                    }
                ) {
                    withAutomaticDeath(5000.0)
                }
            }
        }
        fun moveBackboardToStack() {
            single("go to stackboard from bak") {
                purePursuitNavigateTo(
                    *when (tapeSide)
                    { //backboard
                        TapeSide.Left -> arrayOf(
                            FieldWaypoint(
                                Pose(-86.85, -34.5, (-90.0).degrees),
                                5.0
                            )
                        )

                        TapeSide.Middle -> arrayOf(
                            FieldWaypoint(
                                Pose(-85.85, -28.0, (-90.0).degrees),
                                10.0
                            )
                        )

                        TapeSide.Right -> arrayOf(
                            FieldWaypoint(
                                Pose(-85.85, -22.5, (-90.0).degrees),
                                20.0
                            )
                        )
                    },
                    ActionWaypoint {
                        opMode.clawSubsystem.toggleExtender(
                            ExtendableClaw.ExtenderState.Deposit,
                            force = true
                        )
                    },
                    FieldWaypoint(
                        Pose(-72.0, -42.0, (180).degrees),
                        25.0
                    ),
                    ActionWaypoint {
                        opMode.elevatorSubsystem.configureElevatorManually(0.0)
                    },
                    FieldWaypoint(
                        Pose(-63.0, -56.0, (90).degrees),
                        25.0
                    ),
                    FieldWaypoint(
                        Pose(-5.0, -56.0, (90).degrees),
                        25.0
                    ),
                    FieldWaypoint(
                        farStackPickup,
                        20.0
                    )
                ) {
                    withAutomaticDeath(5000.0)
                }
            }
        }

        spikeMark(opMode, tapeSide)

        single("prep for stak") {
            opMode.clawSubsystem.updateClawState(
                ExtendableClaw.ClawStateUpdate.Right,
                ExtendableClaw.ClawState.MosaicFix,
            )
            opMode.clawSubsystem.toggleExtender(ExtendableClaw.ExtenderState.Deposit, force = true)
            opMode.elevatorSubsystem.configureElevatorManually(stack5)

            Thread.sleep(500L)
        }

        single("go to stak") {

            when (tapeSide)
            {
                TapeSide.Right ->
                {
                    purePursuitNavigateTo(
                        FieldWaypoint(
                            Pose(-1.5, -23.0, (-35).degrees),
                            10.0
                        ),
                        FieldWaypoint(
                            Pose(0.0, -48.0, (0).degrees),
                            10.0
                        )
                    )
                    opMode.clawSubsystem.toggleExtender(
                        ExtendableClaw.ExtenderState.Intake,
                        force = true
                    )
                    purePursuitNavigateTo(
                        FieldWaypoint(
                            Pose(0.0, -48.0, (0).degrees),
                            10.0
                        ),
                        FieldWaypoint(
                            Pose(7.5, -48.0, 90.degrees),
                            5.0
                        )
                    ) {
                        withCustomMaxTranslationalSpeed(0.4)
                        withCustomMaxRotationalSpeed(0.4)
                    }
                    Thread.sleep(750)
                    navigateTo(farStackPickup) {
                        withCustomMaxTranslationalSpeed(0.3)
                        withCustomMaxRotationalSpeed(0.3)
                    }
                }

                TapeSide.Middle ->
                {
                    purePursuitNavigateTo(
                        FieldWaypoint(
                            Pose(0.0, -25.0, 0.degrees),
                            20.0
                        ),
                        FieldWaypoint(
                            Pose(9.0, -25.0, 45.degrees),
                            7.0
                        ),
                        FieldWaypoint(
                            Pose(7.5, -48.0, 90.degrees),
                            10.0
                        )
                    )

                    opMode.clawSubsystem.toggleExtender(
                        ExtendableClaw.ExtenderState.Intake,
                        force = true
                    )
                    Thread.sleep(750)
                    navigateTo(
                        farStackPickup
                    ) {
                        withCustomMaxTranslationalSpeed(0.4)
                    }
                }

                TapeSide.Left ->
                {
                    purePursuitNavigateTo(
                        FieldWaypoint(
                            Pose(0.0, -20.0, 35.degrees),
                            20.0
                        ),
                        FieldWaypoint(
                            Pose(0.0, -30.0, 0.degrees),
                            10.0
                        ),
                        FieldWaypoint(
                            Pose(0.0, -49.0, 0.degrees),
                            10.0
                        ),
                        FieldWaypoint(
                            Pose(-7.0, -49.0, 90.degrees),
                            20.0
                        )
                    ) {
                        withAutomaticDeath(5000.0)
                    }
                    opMode.clawSubsystem.toggleExtender(
                        ExtendableClaw.ExtenderState.Intake,
                        force = true
                    )
                    Thread.sleep(750)
                    navigateTo(
                        farStackPickup
                    ) {
                        withCustomMaxTranslationalSpeed(0.35)
                        withCustomMaxRotationalSpeed(0.35)
                    }
                }
            }
        }

        single("Intake from the stack") {

            opMode.clawSubsystem.updateClawState(
                ExtendableClaw.ClawStateUpdate.Both,
                ExtendableClaw.ClawState.Closed,
                force = true
            )
            Thread.sleep(750)
            opMode.clawSubsystem.toggleExtender(ExtendableClaw.ExtenderState.Intermediate)
        }

        moveStackToBackBoard()

        single("drop on backboard") {
            dropPixels(opMode)
        }

        single("move back to stack") {
            moveBackboardToStack()
        }

        single("prep for stak") {
            opMode.clawSubsystem.updateClawState(
                ExtendableClaw.ClawStateUpdate.Right,
                ExtendableClaw.ClawState.MosaicFix,
            )
            Thread.sleep(500L)
            opMode.clawSubsystem.toggleExtender(ExtendableClaw.ExtenderState.Deposit, force = true)
            opMode.elevatorSubsystem.configureElevatorManually(stack3)

            Thread.sleep(500L)
        }

        single("Intake from the stack") {

            opMode.clawSubsystem.updateClawState(
                ExtendableClaw.ClawStateUpdate.Both,
                ExtendableClaw.ClawState.Closed,
                force = true
            )
            Thread.sleep(750)
            opMode.clawSubsystem.toggleExtender(ExtendableClaw.ExtenderState.Intermediate)
        }

        moveStackToBackBoard()

        single("drop on backboard") {
            dropPixels(opMode)
        }

        single("park") {
            opMode.elevatorSubsystem.configureElevatorManually(0.0)
            purePursuitNavigateTo(
                when (tapeSide)
                {
                    TapeSide.Left ->
                        FieldWaypoint(
                            Pose(redBoardX, -41.0, (-90.0).degrees), 4.0
                        )

                    TapeSide.Middle ->
                        FieldWaypoint(
                            Pose(redBoardX, -28.0, (-90.0).degrees), 4.0
                        )

                    TapeSide.Right ->
                        FieldWaypoint(
                            Pose(redBoardX, -22.5, (-90.0).degrees), 4.0
                        )
                },
                FieldWaypoint(parkMiddle, 10.0)
            ) {
                withCustomMaxTranslationalSpeed(0.5)
            }
        }

    }
)