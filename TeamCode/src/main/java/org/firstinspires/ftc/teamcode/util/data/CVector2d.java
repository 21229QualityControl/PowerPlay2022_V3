package org.firstinspires.ftc.teamcode.util.data;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

/**
 * Dashboard configurable Vector2d
 */
public class CVector2d {
    public double x, y;

    public CVector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public CVector2d(double x) {
        this(x, 0);
    }
    public CVector2d() {
        this(0, 0);
    }
    public CVector2d(Vector2d vector2d) {
        this(vector2d.getX(), vector2d.getY());
    }
    public static CVector2d polar(double r, double theta) {
        return new CVector2d(r * cos(theta), r * sin(theta));
    }

    public double norm() {
        return Math.sqrt(x * x + y * y);
    }

    public double angle() {
        return Angle.norm(atan2(y, x));
    }

    public double angleBetween(Vector2d other) {
        return Math.acos(this.dot(other) / (norm() * other.norm()));
    }
    public double angleBetween(CVector2d other) {
        return Math.acos(this.dot(other) / (norm() * other.norm()));
    }

    public Vector2d plus(Vector2d other) {
        return new Vector2d(x + other.getX(), y + other.getY());
    }
    public Vector2d plus(CVector2d other) {
        return new Vector2d(x + other.getX(), y + other.getY());
    }

    public Vector2d minus(Vector2d other) {
        return new Vector2d(x - other.getX(), y - other.getY());
    }
    public Vector2d minus(CVector2d other) {
        return new Vector2d(x - other.getX(), y - other.getY());
    }

    public Vector2d times(double scalar) {
        return new Vector2d(scalar * x, scalar * y);
    }

    public Vector2d div(double scalar) {
        return new Vector2d(x / scalar, y / scalar);
    }

    public Vector2d unaryMinus() {
        return new Vector2d(-x, -y);
    }

    public double dot(Vector2d other) {
        return x * other.getX() + y * other.getY();
    }
    public double dot(CVector2d other) {
        return x * other.getX() + y * other.getY();
    }

    public double distanceTo(Vector2d other) {
        return (this.minus(other)).norm();
    }
    public double distanceTo(CVector2d other) {
        return (this.minus(other)).norm();
    }

    public Vector2d projectOnto(Vector2d other) {
        return other.times(this.dot(other) / other.dot(other));
    }
    public Vector2d projectOnto(CVector2d other) {
        return other.times(this.dot(other) / other.dot(other));
    }

    public Vector2d rotated(double angle) {
        double newX = x * cos(angle) - y * sin(angle);
        double newY = x * sin(angle) + y * cos(angle);
        return new Vector2d(newX, newY);
    }

    public boolean episilonEquals(Vector2d other) {
        return epsilonEqualsNum(x, other.getX()) && epsilonEqualsNum(y, other.getY());
    }
    public boolean episilonEquals(CVector2d other) {
        return epsilonEqualsNum(x, other.getX()) && epsilonEqualsNum(y, other.getY());
    }

    private boolean epsilonEqualsNum(double a, double b) {
        return abs(a - b) < 1e-6;
    }

    @Override
    public String toString() {
        return String.format("(%.3f, %.3f)", x, y);
    }

    public Vector2d asVector2d() {
        return new Vector2d(x, y);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
