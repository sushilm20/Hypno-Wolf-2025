package org.robotics.robotics.xdk.teamcode.autonomous.position.guidedvectorfield;

import com.acmerobotics.dashboard.config.Config;

import org.jetbrains.annotations.NotNull;
import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose;

// realtime bezier guided vector field navigation
@Config
public class GuidedVectorField {

    public static double CORRECTION_DISTANCE = 100;
    public static double SAVING_THROW_DISTANCE = 100;
    public static int SAMPLE_DENSITY = 100;

    private final @NotNull CubicBezierCurve curve;
    public GuidedVectorField(@NotNull CubicBezierCurve curve) {
        this.curve = curve;
    }

    public @NotNull Pose calculateGuidanceVector(@NotNull Pose currentPose) {
        Vector2D currentLocation = new Vector2D(currentPose.x, currentPose.y);
        double closestT = findClosestPoint(curve, currentLocation);
        Vector2D closestPoint = curve.calculate(closestT);
        Vector2D curveDerivative = curve.derivative(closestT);
        Vector2D robotToClosestPoint = closestPoint.subtract(currentLocation);
        Vector2D endPoint = curve.calculate(1);
        double directPursuitThreshold = 1;
        {
            for (double i = 1; i >= 0; i -= 1 / 100.0) {
                double dist = endPoint.subtract(curve.calculate(i)).getMagSq();
                if (dist > SAVING_THROW_DISTANCE) {
                    directPursuitThreshold = i;
                    break;
                }
            }
        }
        Vector2D robotToEnd = endPoint.subtract(currentLocation);
        double correctionFactor = Math.min(1, robotToClosestPoint.getMagnitude() / CORRECTION_DISTANCE);
        double movementDirection = hlerp(curveDerivative.getHeading(), robotToClosestPoint.getHeading(), correctionFactor);
        if ((closestT == 1 && Math.abs(currentLocation.subtract(closestPoint).getHeading() - curveDerivative.getHeading()) <= 0.5 * Math.PI) ||
            closestT >= directPursuitThreshold) {
            movementDirection = endPoint.subtract(currentLocation).getHeading();
        }

        Vector2D movementVector = new Vector2D(Math.cos(movementDirection), Math.sin(movementDirection));
        double speed = 1;
        // i'd like to change this logic to distance along the path.
        // maybe use something like euler's method?
        // split the curve into intervals, calculate delta vectors
        // between intervals, store in a lookup table
        if (robotToEnd.getMagnitude() < 200) {
            speed = lerp(0.2, speed, robotToEnd.getMagnitude() / 200);
        }

        movementVector = movementVector.scalarMultiply(speed);
        //System.out.println(String.format("Navigation Calculation Took %.3fms", (System.nanoTime() - startTime) / 1e6));
        return new Pose(movementVector.getX(), movementVector.getY(), movementVector.getHeading());
    }

    /*
    Hlerp - heading lerp.
    Interpolates between a and b, taking the shortest path across the range
    [-pi, pi] assuming the input range is continuous across said range.

    Say a = -0.9pi, b = 0.9pi. traditional lerp would rotate counterclockwise,
    passing through 0 at t = 0.5. Hlerp will rotate clockwise, passing through
    +/- pi at t = 0.5.

    Hlerp is used to avoid an edge case where the movement direction
    passes through +/- pi as it transitions from to-path correction to
    on-path guidance.
     */
    public double hlerp(double a, double b, double t) {
        double diff = b - a;
        diff %= 2 * Math.PI;
        if (Math.abs(diff) > Math.PI) {
            if (diff > 0) {
                diff -= 2 * Math.PI;
            } else {
                diff += 2 * Math.PI;
            }
        }
        return a + t * diff;
    }

    private double lerp(double a, double b, double t) {
        return (1 - t) * a + t * b;
    }

    public double findClosestPoint(CubicBezierCurve curve, Vector2D point) {
        //long startTime = System.nanoTime();
        double minT = -1;
        double minDist = Double.POSITIVE_INFINITY;
        for (int i = 0; i < SAMPLE_DENSITY + 1; i++) {
            double t = i / (double) SAMPLE_DENSITY;
            double dist = calculateMinimizationFunction(curve, t, point);
            if (dist < minDist) {
                minDist = dist;
                minT = t;
            }
        }
        //System.out.println(String.format("Calculated closest point in %.3f ms", (System.nanoTime() - startTime) / 1e6));
        return minT;
    }

    private double calculateMinimizationFunction(CubicBezierCurve curve, double t, Vector2D point) {
        return curve.calculate(t).subtract(point).getMagSq();
    }
}