package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Interpolates a value to another with a given speed or duration
 *
 * Essentially wraps a servo position and gives the illusion of following the servo
 */
public class Interpolator {
    private ElapsedTime timer;

    private double current;

    private boolean interpolationActive;
    private double interpolationStart;
    private double interpolationEnd;
    private double interpolationEndTime;
    private double interpolationRate;

    private double extraWait = 0;


    public Interpolator(double start) {
        interpolationActive = false;
        setLocation(start);
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }
    public Interpolator() {
        this(0);
    }

    public synchronized void setLocation(double location) {
        this.current = location;
        interpolationActive = false;
    }

    public void update() {
        if (interpolationActive) {
            current = interpolationStart + (interpolationEnd - interpolationStart) * Range.clip(timer.milliseconds() / interpolationEndTime, 0, 1);
        }
    }

    public synchronized double getCurrentLocation() {
        return current;
    }

    public synchronized double getTarget() {
        if (interpolationActive) {
            return interpolationEnd;
        }
        return current;
    }

    public synchronized double getStart() {
        if (interpolationActive) {
            return interpolationStart;
        }
        return current;
    }

    public synchronized void interpolateWithDuration(double end, double totalMs) {
        if (interpolationActive && epsilonEquals(end, interpolationEnd) && epsilonEquals(durationToRate(getCurrentLocation(), end, totalMs), interpolationRate)) {
            return; // return if settings are the same
        }
        update();
        interpolationStart = getCurrentLocation();
        interpolationEnd = end;
        interpolationEndTime = totalMs;
        interpolationRate = durationToRate(interpolationStart, interpolationEnd, interpolationEndTime);
        interpolationActive = true;
        timer.reset();
    }

    public synchronized void interpolateWithSpeed(double end, double unitPerSec) {
        if (interpolationActive && epsilonEquals(end, interpolationEnd) && epsilonEquals(unitPerSec, interpolationRate)) {
            return; // return if settings are the same
        }
        update();
        interpolationStart = getCurrentLocation();
        interpolationEnd = end;
        interpolationEndTime = rateToDuration(interpolationStart, interpolationEnd, unitPerSec);
        interpolationRate = unitPerSec;
        interpolationActive = true;
        timer.reset();
    }

    public synchronized void setExtraWait(double extraWaitMS) {
        extraWait = extraWaitMS;
    }

    public synchronized double getTimeLeft() {
        if (interpolationActive) {
            return interpolationEndTime + extraWait - timer.milliseconds();
        }
        return 0;
    }

    public synchronized boolean isIdle() {
        if (interpolationActive) {
            return timer.milliseconds() >= interpolationEndTime + extraWait;
        }
        return true;
    }

    public String getDebugInfo() {
        return String.format("%s %.5f @ (%.2f / %.2f ms) of [%.5f -> %.5f]", interpolationActive ? "Active" : "Inactive", current, timer.milliseconds(), interpolationEndTime, interpolationStart, interpolationEnd);
    }

    private boolean epsilonEquals(double a, double b) {
        return Math.abs(a - b) < 1e-6;
    }

    private double rateToDuration(double start, double end, double unitPerSec) {
        return Math.abs(start - end) * 1000 / unitPerSec;
    }
    private double durationToRate(double start, double end, double duration) {
        return Math.abs(start - end) * 1000 / duration;
    }
}
