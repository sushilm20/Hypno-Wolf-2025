package org.robotics.robotics.xdk.teamcode.autonomous.purepursuit;

import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Point;

public class PurePursuitUtil {
    public static Point lineCircleIntersection(Point pointA, Point pointB, Point center, double radius) {
        double baX = pointB.x - pointA.x;
        double baY = pointB.y - pointA.y;
        double caX = center.x - pointA.x;
        double caY = center.y - pointA.y;

        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;

        double pBy2 = bBy2 / a;
        double q = c / a;
        double disc = pBy2 * pBy2 - q;

        if (disc < 0) {
            // No intersection, return pointA or handle it accordingly
            return pointA;
        }

        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double x1 = pointA.x + baX * abScalingFactor1;
        double y1 = pointA.y + baY * abScalingFactor1;

        if (disc == 0) {
            return new Point(x1, y1);
        }

        double abScalingFactor2 = -pBy2 - tmpSqrt;
        double x2 = pointA.x + baX * abScalingFactor2;
        double y2 = pointA.y + baY * abScalingFactor2;

        double dist1 = Math.hypot(pointB.x - x1, pointB.y - y1);
        double dist2 = Math.hypot(pointB.x - x2, pointB.y - y2);

        return dist1 > dist2 ? new Point(x2, y2) : new Point(x1, y1);
    }
}
