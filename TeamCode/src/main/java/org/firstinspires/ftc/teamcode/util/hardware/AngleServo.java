package org.firstinspires.ftc.teamcode.util.hardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class for a servo that uses degrees instead of 0-1 position
 */
public class AngleServo {
    private Servo servo;

    private double referencePos1, referencePos2;
    private double referenceDeg1, referenceDeg2;

    private double degreeAtZero, degreeAtOne;
    private double degreeMin, degreeMax;

    private Direction direction;

    private double position, angle;

    public enum Direction {
        CLOCKWISE,
        COUNTERCLOCKWISE
    }

    public AngleServo(Servo servo, double degreeAtZero, double degreeAtOne, Direction direction) {
        this(servo, 0, degreeAtZero, 1, degreeAtOne, direction);
    }

    public AngleServo(Servo servo, double referencePos1, double referenceDeg1, double referencePos2, double referenceDeg2, Direction direction) {
        this.servo = servo;

        this.direction = direction;

        setReferences(referencePos1, referenceDeg1, referencePos2, referenceDeg2);

        if (direction.equals(Direction.CLOCKWISE)) {
            degreeMin = degreeAtOne;
            degreeMax = degreeAtZero;
        } else {
            degreeMin = degreeAtZero;
            degreeMax = degreeAtOne;
        }

        matchPosition(0); // check for error
    }

    public Servo getServo() {
        return servo;
    }

    public void setReferences(double referencePos1, double referenceDeg1, double referencePos2, double referenceDeg2) {
        this.referencePos1 = referencePos1;
        this.referencePos2 = referencePos2;
        this.referenceDeg1 = referenceDeg1;
        this.referenceDeg2 = referenceDeg2;
        recalculateConfig();
    }
    public void setReference1(double position, double correspondingDegree) {
        this.referencePos1 = position;
        this.referenceDeg1 = correspondingDegree;

        recalculateConfig();
    }
    public void setReference2(double position, double correspondingDegree) {
        this.referencePos2 = position;
        this.referenceDeg2 = correspondingDegree;

        recalculateConfig();
    }

    private void recalculateConfig() {
        double derivative = (referenceDeg2 - referenceDeg1) / (referencePos2 - referencePos1); // (45 - 0) / (0.35 - 0.5) = -300
        degreeAtZero = referenceDeg1 - referencePos1 * derivative; // 0 - 0.5(-300) = 150
        degreeAtOne = degreeAtZero + derivative; // -150
    }

    public void scaleRange(double min, double max) {
        this.servo.scaleRange(min, max);
    }

    public void setDegreeMin(double degreeMin) {
        this.degreeMin = degreeMin;
    }
    public void setDegreeMax(double degreeMax) {
        this.degreeMax = degreeMax;
    }

    public double getDegreeMin() {
        return Math.max(degreeMin, Math.min(degreeAtOne, degreeAtZero));
    }
    public double getDegreeMax() {
        return Math.min(degreeMax, Math.max(degreeAtOne, degreeAtZero));
    }

    public void setAngle(double targetAngle) {
        targetAngle = Math.min(Math.max(degreeMin, targetAngle), degreeMax);
        double targetPosition = matchPosition(targetAngle);
        targetPosition = Math.min(Math.max(0, targetPosition), 1);
        servo.setPosition(targetPosition);
        this.position = targetPosition;
        if (Math.abs(servo.getPosition() - targetPosition) > 1e-6) Log.d("AngleServo", "Set position not assigned for " + targetAngle + "°. " + targetPosition + " -> " + servo.getPosition());

        if (position == 0) this.angle = degreeAtZero;
        else if (position == 1) this.angle = degreeAtOne;
        else this.angle = targetAngle;
    }

    public double getPosition() {
        return this.position;
    }

    public double getAngle() {
        return this.angle;
    }

    private double matchPosition(double targetDeg) {
        return matchPosition(degreeAtZero, degreeAtOne, targetDeg, direction.equals(Direction.CLOCKWISE));
    }

    /**
     * Made by myself
     *
     * @param deg0      the angle when position is set to 0
     * @param deg1      the angle when position is set to 1
     * @param targetDeg the target angle where we want the corresponding position
     * @param isCW      whether or not to turn clockwise(true) to get from rad0 to rad1
     * @return the corresponding position for the target angle
     */
    private static double matchPosition(double deg0, double deg1, double targetDeg, boolean isCW) throws RuntimeException {
        if (isCW && !(deg1 <= deg0)) {
            throw new RuntimeException("Rotating CW but does not fulfill [0:" + deg0 + " >= 1:" + deg1 + "]");
        }
        if (!isCW && !(deg0 <= deg1)) {
            throw new RuntimeException("Rotating CCW but does not fulfill [1:" + deg1 + " >= 0:" + deg0 + "]");
        }

        return (targetDeg - deg0) / (deg1 - deg0);
    }

    /**
     * @return the degree the servo is capable of achieving
     */
    public double cropDegree(double rawDegree) {
        double rawPos = matchPosition(rawDegree);
        if (rawPos < 0) return degreeAtZero;
        if (rawPos > 1) return degreeAtOne;

        return rawDegree;
    }
}