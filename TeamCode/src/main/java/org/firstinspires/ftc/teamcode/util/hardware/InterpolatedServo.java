package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Interpolator;

public class InterpolatedServo {
    private ServoImplEx servo;
    private Interpolator interpolator;
    private double posPerSec;

    public InterpolatedServo(ServoImplEx servo) {
        this.servo = servo;
        this.interpolator = new Interpolator(0.5);
        this.posPerSec = 1;
    }

    public void update() {
        interpolator.update();
        servo.setPosition(interpolator.getCurrentLocation());
    }

    public void setPosition(double position) {
        position = Range.clip(position, 0, 1);
        interpolator.interpolateWithSpeed(position, posPerSec);
    }

    public double getPosition() {
        return interpolator.getCurrentLocation();
    }

    public double getTargetPosition() {
        return interpolator.getTarget();
    }

    public void setSpeed(double posPerSec) {
        interpolator.interpolateWithSpeed(interpolator.getTarget(), posPerSec);
    }
}
