package org.riverdell.robotics.autonomous.movement.guidedvectorfield;

public class Vector2D {
    private double x, y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector2D polar(double r, double t) {
        return new Vector2D(r * Math.cos(t), r * Math.sin(t));
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getHeading() {
        return Math.atan2(y, x);
    }

    public double getMagnitude() {
        return Math.hypot(x, y);
    }

    public double getMagSq() {
        return x * x + y * y;
    }

    // Operations

    public Vector2D add(Vector2D other) {
        return add(this, other);
    }

    public Vector2D subtract(Vector2D other) {
        return subtract(this, other);
    }

    public Vector2D scalarMultiply(double scalar) {
        return scalarMultiply(this, scalar);
    }

    public Vector2D scalarDivide(double scalar) {
        return scalarDivide(this, scalar);
    }

    public static Vector2D add(Vector2D a, Vector2D b) {
        return new Vector2D(a.x + b.x, a.y + b.y);
    }

    public static Vector2D subtract(Vector2D a, Vector2D b) {
        return new Vector2D(a.x - b.x, a.y - b.y);
    }

    public static Vector2D scalarMultiply(Vector2D vec, double scalar) {
        return new Vector2D(vec.x * scalar, vec.y * scalar);
    }

    public static Vector2D scalarDivide(Vector2D vec, double scalar) {
        return new Vector2D(vec.x / scalar, vec.y / scalar);
    }

    public static Vector2D slerp(Vector2D a, Vector2D b, double t) {
        double aMag = a.getMagnitude();
        double aHead = a.getHeading();
        double bMag = b.getMagnitude();
        double bHead = b.getHeading();
        return polar((1 - t) * aMag + t * bMag, (1 - t) * aHead + t * bHead);
    }
}