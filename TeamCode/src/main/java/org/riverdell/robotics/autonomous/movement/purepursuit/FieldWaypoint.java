package org.riverdell.robotics.autonomous.movement.purepursuit;
import org.riverdell.robotics.autonomous.movement.geometry.Point;
import org.riverdell.robotics.autonomous.movement.geometry.Pose;

import java.util.Locale;
import java.util.UUID;

public class FieldWaypoint implements WaypointLike {
    private final Type type;
    private final Point point;
    private final double radius;
    public final String id = UUID.randomUUID().toString();

    public FieldWaypoint(Point point, double radius) {
        this.type = point instanceof Pose ? Type.POSE : Type.POINT;
        this.point = point;
        this.radius = radius;
    }

    public Type getType() {
        return this.type;
    }

    public Point getPoint() {
        return this.point;
    }

    public double getRadius() {
        return this.radius;
    }

    public enum Type {
        POINT, POSE
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%s %s %.2f", getType(), getPoint(), getRadius());
    }
}