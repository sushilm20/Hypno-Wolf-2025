package org.riverdell.robotics.utilities.motionprofile;

// thx cookie bots
public class AsymmetricMotionProfile {
    public double initialPosition;
    public double finalPosition;
    public double distance;
    public double t1, t2, t3;
    public double totalTime;
    public double t1_stop_position;
    public double max_velocity;
    public double t2_stop_position;
    public boolean flipped = false;
    public double originalPos = 0;

    public MotionProfileConstraints constraints;

    public AsymmetricMotionProfile(double initialPosition, double finalPosition, MotionProfileConstraints constraints) {
        if (finalPosition < initialPosition) {
            flipped = true;
            this.originalPos = initialPosition;
            double temp = initialPosition;
            initialPosition = finalPosition;
            finalPosition = temp;
        }
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        this.distance = finalPosition - initialPosition;
        this.constraints = constraints;

        t1 = constraints.getVelocity() / constraints.getAcceleration();
        t3 = constraints.getVelocity() / constraints.getDeceleration();
        t2 = Math.abs(distance) / constraints.getVelocity() - (t1 + t3) / 2;

        if (t2 < 0) {
            this.t2 = 0;

            double a = (constraints.getAcceleration() / 2) * (1 - constraints.getAcceleration() / -constraints.getDeceleration());
            double c = -distance;

            t1 = Math.sqrt(-4 * a * c) / (2 * a);
            t3 = -(constraints.getAcceleration() * t1) / -constraints.getDeceleration();
            t1_stop_position = (constraints.getAcceleration() * Math.pow(t1, 2)) / 2;

            max_velocity = constraints.getAcceleration() * t1;

            t2_stop_position = t1_stop_position;
        } else {
            max_velocity = constraints.getVelocity();
            t1_stop_position = (constraints.getVelocity() * t1) / 2;
            t2_stop_position = t1_stop_position + t2 * max_velocity;
        }

        totalTime = t1 + t2 + t3;
    }

    public ProfileState calculate(final double time) {
        double position, velocity, acceleration, stage_time;
        if (time <= t1) {
            stage_time = time;
            acceleration = constraints.getAcceleration();
            velocity = acceleration * stage_time;
            position = velocity * stage_time / 2;
        } else if (time <= t1 + t2) {
            stage_time = time - t1;
            acceleration = 0;
            velocity = constraints.getVelocity();
            position = t1_stop_position + stage_time * velocity;
        } else if (time <= totalTime) {
            stage_time = time - t1 - t2;
            acceleration = -constraints.getDeceleration();
            velocity = max_velocity - stage_time * constraints.getDeceleration();
            position = t2_stop_position + (max_velocity + velocity) / 2 * stage_time;
        } else {
            acceleration = 0;
            velocity = 0;
            position = finalPosition;
        }

        if (time <= totalTime) {
            if (flipped) {
                return new ProfileState(originalPos - position, velocity, acceleration);
            } else {
                return new ProfileState(originalPos + position, velocity, acceleration);
            }
        } else {
            if (flipped) {
                return new ProfileState(initialPosition, velocity, acceleration);
            } else {
                return new ProfileState(originalPos + position, velocity, acceleration);
            }
        }
    }
}