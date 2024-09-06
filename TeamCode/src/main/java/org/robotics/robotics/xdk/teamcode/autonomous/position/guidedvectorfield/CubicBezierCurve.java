package org.robotics.robotics.xdk.teamcode.autonomous.position.guidedvectorfield;

public class CubicBezierCurve {

    private Vector2D p0, p1, p2, p3;

    public CubicBezierCurve(Vector2D p0, Vector2D p1, Vector2D p2, Vector2D p3) {
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
    }

    public Vector2D getP0() {
        return p0;
    }

    public void setP0(Vector2D p0) {
        this.p0 = p0;
    }

    public Vector2D getP1() {
        return p1;
    }

    public void setP1(Vector2D p1) {
        this.p1 = p1;
    }

    public Vector2D getP2() {
        return p2;
    }

    public void setP2(Vector2D p2) {
        this.p2 = p2;
    }

    public Vector2D getP3() {
        return p3;
    }

    public void setP3(Vector2D p3) {
        this.p3 = p3;
    }

    public Vector2D calculate(double t) {
        // (1 - t)^3 * P0 + 3 * t * (1 - t)^2 * P1 + 3 * t^2 * (1 - t) * P2 + t^3 * P3
        double w = 1 - t;
        Vector2D firstTerm = p0.scalarMultiply(w * w * w);
        Vector2D secondTerm = p1.scalarMultiply(3 * t * w * w);
        Vector2D thirdTerm = p2.scalarMultiply(3 * t * t * w);
        Vector2D fourthTerm = p3.scalarMultiply(t * t * t);
        return firstTerm.add(secondTerm).add(thirdTerm).add(fourthTerm);
    }

    public Vector2D derivative(double t) {
        double w = 1 - t;
        Vector2D firstTerm = p1.subtract(p0).scalarMultiply(3 * w * w);
        Vector2D secondTerm = p2.subtract(p1).scalarMultiply(6 * w * t);
        Vector2D thirdTerm = p3.subtract(p2).scalarMultiply(3 * t * t);
        return firstTerm.add(secondTerm).add(thirdTerm);
    }

    public double slope(double t) {
        Vector2D dt = derivative(t);
        return dt.getY() / dt.getX();
    }

    public double heading(double t) {
        return derivative(t).getHeading();
    }
}