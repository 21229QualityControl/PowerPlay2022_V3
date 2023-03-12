package org.firstinspires.ftc.teamcode.util.data;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

/**
 * Dashboard configurable Pose2d
 */
public class CPose2d {
    public double x, y, degrees; // heading is configured in degrees but is accessible only in radians from the code

    public CPose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.degrees = Math.toDegrees(heading);
    }
    public CPose2d(double x, double y) {
        this(x, y, 0);
    }
    public CPose2d(double x) {
        this(x, 0, 0);
    }
    public CPose2d() {
        this(0, 0, 0);
    }
    public CPose2d(Vector2d pos, double heading) {
        this(pos.getX(), pos.getY(), heading);
    }
    public CPose2d(CVector2d pos, double heading) {
        this(pos.getX(), pos.getY(), heading);
    }
    public CPose2d(Pose2d pose2d) {
        this(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    public Vector2d vec() {
        return new Vector2d(x, y);
    }

    public Vector2d headingVec() {
        return new Vector2d(cos(Math.toRadians(degrees)), sin(Math.toRadians(degrees)));
    }

    public Pose2d plus(Pose2d other) {
        return new Pose2d(x + other.getX(), y + other.getY(), Math.toRadians(degrees) + other.getHeading());
    }
    public Pose2d plus(CPose2d other) {
        return new Pose2d(x + other.getX(), y + other.getY(), Math.toRadians(degrees) + other.getHeading());
    }

    public Pose2d minus(Pose2d other) {
        return new Pose2d(x - other.getX(), y - other.getY(), Math.toRadians(degrees) - other.getHeading());
    }
    public Pose2d minus(CPose2d other) {
        return new Pose2d(x - other.getX(), y - other.getY(), Math.toRadians(degrees) - other.getHeading());
    }

    public Pose2d times(double scalar) {
        return new Pose2d(scalar * x, scalar * y, scalar * Math.toRadians(degrees));
    }

    public Pose2d div(double scalar) {
        return new Pose2d(x / scalar, y / scalar, Math.toRadians(degrees) / scalar);
    }

    public Pose2d unaryMinus() {
        return new Pose2d(-x, -y, -Math.toRadians(degrees));
    }

    public boolean epsilonEquals(Pose2d other) {
        return epsilonEqualsNum(x, other.getX()) && epsilonEqualsNum(y, other.getY());
    }
    public boolean epsilonEquals(CPose2d other) {
        return epsilonEqualsNum(x, other.getX()) && epsilonEqualsNum(y, other.getY());
    }

    public boolean epsilonEqualsHeading(Pose2d other) {
        return epsilonEqualsNum(x, other.getX()) && epsilonEqualsNum(y, other.getY()) && epsilonEqualsNum(Angle.normDelta(Math.toRadians(degrees) - other.getHeading()), 0);
    }
    public boolean epsilonEqualsHeading(CPose2d other) {
        return epsilonEqualsNum(x, other.getX()) && epsilonEqualsNum(y, other.getY()) && epsilonEqualsNum(Angle.normDelta(Math.toRadians(degrees) - other.getHeading()), 0);
    }

    private boolean epsilonEqualsNum(double a, double b) {
        return abs(a - b) < 1e-6;
    }

    @Override
    public String toString() {
        return String.format("(%.3f, %.3f, %.3f°)", x, y, degrees);
    }

    public Pose2d asPose2d() {
        return new Pose2d(x, y, Math.toRadians(degrees));
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return Math.toRadians(degrees);
    }
}
