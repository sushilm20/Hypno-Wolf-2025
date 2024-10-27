package org.riverdell.robotics.autonomous.movement;

import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.riverdell.robotics.autonomous.HypnoticAuto;
import org.riverdell.robotics.autonomous.movement.geometry.Pose;

public class MecanumTranslations {

    public static DrivetrainUpdates getPowers(Pose pose) {
        return getPowers(pose, 0);
    }

    public static DrivetrainUpdates getPowers(double strafeSpeed, double forwardSpeed,
                              double turnSpeed, double gyroAngle) {
        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed).rotateBy(-gyroAngle);

        strafeSpeed = Range.clip(input.getX(), -1, 1);
        forwardSpeed = Range.clip(input.getY(), -1, 1);
        turnSpeed = Range.clip(turnSpeed, -1, 1);

        double[] wheelSpeeds = new double[4];

        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] = forwardSpeed + strafeSpeed + turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] = forwardSpeed - strafeSpeed - turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] = (forwardSpeed - strafeSpeed + turnSpeed);
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] = (forwardSpeed + strafeSpeed - turnSpeed);
        // 1.06, 1.04

        // feedforward & voltage comp
        double correction = 12 / HypnoticAuto.getInstance().getRobot().getDrivetrain().voltage();
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = Math.abs(wheelSpeeds[i]) < 0.01 ?
                    wheelSpeeds[i] * correction :
                    (wheelSpeeds[i] + Math.signum(wheelSpeeds[i]) * 0.085) * correction;
        }

        double max = 1;
        for (double wheelSpeed : wheelSpeeds) max = Math.max(max, Math.abs(wheelSpeed));

        if (max > 1) {
            wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kBackRight.value] /= max;
        }

        return new DrivetrainUpdates(
                wheelSpeeds[0],
                wheelSpeeds[1],
                wheelSpeeds[2],
                wheelSpeeds[3]
        );
    }

    public static DrivetrainUpdates getPowers(Pose pose, double angle) {
        return getPowers(pose.x, pose.y, pose.getHeading(), angle);
    }
}
