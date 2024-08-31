package org.robotics.robotics.xdk.teamcode.autonomous.position;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.robotics.robotics.xdk.teamcode.autonomous.AbstractAutoPipeline;
import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose;

import io.liftgate.robotics.mono.pipeline.RootExecutionGroup;

@Config
public class PositionCommand {
    private static final double K_STATIC = 1.85;
    private static final DrivetrainUpdates ZERO = new DrivetrainUpdates(0.0, 0.0, 0.0, 0.0);

    public static double xP = 0.07;
    public static double xD = 0.012;

    public static double yP = 0.07;
    public static double yD = 0.012;

    public static double hP = 1;
    public static double hD = 0.075;

    public static double ALLOWED_TRANSLATIONAL_ERROR = 0.75;
    public static double ALLOWED_HEADING_ERROR = 0.02;
    public static double STABLE_MS = 100;

    public PIDFController xController = new PIDFController(xP, 0.0, xD, 0);
    public PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime stable = new ElapsedTime();

    private double automaticDeathMillis = 2500;
    private final AbstractAutoPipeline drivetrain;
    private final Pose targetPose;

    public void withAutomaticDeath(double automaticDeathMillis) {
        this.automaticDeathMillis = automaticDeathMillis;
    }

    private double maxTranslationalSpeed = 1.0;
    private double maxRotationalSpeed = 1.0;

    private final RootExecutionGroup executionGroup;

    public PositionCommand(final Pose targetPose, final RootExecutionGroup executionGroup) {
        this.drivetrain = AbstractAutoPipeline.getInstance();
        this.targetPose = targetPose;
        this.executionGroup = executionGroup;
    }

    private Supplies<Pose, Pose> targetPoseSupplier = (pose) -> new Pose();
    public void setMaxRotationalSpeed(double maxRotationalSpeed) {
        this.maxRotationalSpeed = maxRotationalSpeed;
    }

    public void setMaxTranslationalSpeed(double maxTranslationalSpeed) {
        this.maxTranslationalSpeed = maxTranslationalSpeed;
    }

    public void supplyCustomTargetPose(Supplies<Pose, Pose> supplier) {
        this.targetPoseSupplier = supplier;
    }

    public void execute() {
        while (true) {
            if (drivetrain.isStopRequested()) {
                executionGroup.terminateMidExecution();
                return;
            }

            Pose robotPose = drivetrain.getLocalizer().getPose();
            Pose targetPose = this.targetPose != null ? this.targetPose : targetPoseSupplier.supply(robotPose);

            if (isFinished(robotPose, targetPose)) {
                break;
            }

            Pose powers = getPower(robotPose, targetPose);
            MecanumTranslations.getPowers(powers).propagate(drivetrain);
        }

        ZERO.propagate(drivetrain);
    }

    private boolean isFinished(Pose robotPose, Pose targetPose) {
        Pose delta = targetPose.subtract(robotPose);

        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR || Math.abs(delta.heading) > ALLOWED_HEADING_ERROR) {
            stable.reset();
        }

        return timer.milliseconds() > automaticDeathMillis || stable.milliseconds() > STABLE_MS;
    }

    public Pose getPower(Pose robotPose, Pose targetPose) {
        double headingError = targetPose.heading - robotPose.heading;
        if (headingError > Math.PI) targetPose.heading -= 2 * Math.PI;
        if (headingError < -Math.PI) targetPose.heading += 2 * Math.PI;

        double xPower = xController.calculate(robotPose.x, targetPose.x);
        double yPower = yController.calculate(robotPose.y, targetPose.y);
        double hPower = hController.calculate(robotPose.heading, targetPose.heading);

        double x_rotated = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_rotated = xPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        hPower = Range.clip(hPower, -maxRotationalSpeed, maxRotationalSpeed);
        x_rotated = Range.clip(x_rotated, -maxTranslationalSpeed / K_STATIC, maxTranslationalSpeed / K_STATIC);
        y_rotated = Range.clip(y_rotated, -maxTranslationalSpeed, maxTranslationalSpeed);

        return new Pose(x_rotated * K_STATIC, y_rotated, hPower);
    }
}
