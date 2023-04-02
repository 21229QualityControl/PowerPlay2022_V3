package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.acmerobotics.roadrunner.util.NanoClock;

/**
 * Kalman Filter implementation of a single-in single-out system.
 *
 * Based off of <a href="https://www.ctrlaltftc.com/advanced/the-kalman-filter">https://www.ctrlaltftc.com/advanced/the-kalman-filter</a>
 */
public class KalmanFilterSISO {
    // State of the Kalman filter (x): Represents estimate
    private double stateEstimate;
    // Model covariance (Q): Uncertainty of the Kalman filter's estimation model
    public double modelUncertainty;
    // Sensor covariance (R): Uncertainty of the measurements from the sensor
    public double sensorUncertainty;
    // Covariance guess (p): Uncertainty of the Kalman filter's estimate
    private double estimateUncertainty;
    // Kalman gain guess (K): How trustworthy the sensor reading is compared to the previous estimate
    private double trustFactor;

    // Previous state of the kalman filter: Previous estimate
    private double previousStateEstimate;
    // Previous covariance guess: Previous uncertainty of the Kalman filter's estimate
    private double previousEstimateUncertainty;
    // Previous measurement: Previous measurement from the sensor
    private double previousMeasurement;
    // Input variable: Difference in reading between two consecutive sensor measurements
    private double sensorDelta;

    private NanoClock clock;
    private double lastUpdateTime;

    /**
     * Constructs a new kalman filter with the given initial state estimate,
     * model uncertainty, and sensor uncertainty.
     *
     * @param initialState The initial state estimate for the filter.
     * @param modelUncertainty The uncertainty of the kalman filter's estimation model.
     * @param sensorUncertainty The uncertainty of the measurements from the sensor.
     */
    public KalmanFilterSISO(double initialState, double modelUncertainty, double sensorUncertainty) {
        this.stateEstimate = initialState;
        this.modelUncertainty = modelUncertainty;
        this.sensorUncertainty = sensorUncertainty;
        reset();
        this.clock = NanoClock.system();
        this.lastUpdateTime = clock.seconds();
    }

    /**
     * Constructs a new Kalman filter with a default modelUncertainty of 0.4 and sensorUncertainty of 0.03
     *
     * @param initialState The initial state estimate for the filter.
     */
    public KalmanFilterSISO(double initialState) {
        this(initialState, 0.4, 0.03);
    }

    /**
     * Resets the kalman filter to its initial state
     */
    public void reset() {
        this.estimateUncertainty = 1;
        this.trustFactor = 1;

        this.previousStateEstimate = this.stateEstimate;
        this.previousEstimateUncertainty = this.estimateUncertainty;
        this.previousMeasurement = this.stateEstimate;
        this.sensorDelta = 0;
    }

    /**
     * Updates the kalman filter with a new sensor reading.
     *
     * @param rawSensorReading The new raw sensor reading to incorporate into the filter.
     */
    public void update(double rawSensorReading) {
        // Get the change in sensor measurements
        double now = clock.seconds();
        sensorDelta = (rawSensorReading - previousMeasurement) / (now - lastUpdateTime); // FIXME: This calculation of sensorDelta introduces major overshoot when this delta should be conservative
        lastUpdateTime = now;

        // Update the state of the filter using the change in raw reading
        stateEstimate = previousStateEstimate + sensorDelta;

        // Update the covariance guess
        Log.d("Kalman", "First Estimate uncertainty " + estimateUncertainty + " -> " + (previousEstimateUncertainty + modelUncertainty));
        estimateUncertainty = previousEstimateUncertainty + modelUncertainty;

        // Update the kalman gain
        Log.d("Kalman", "Trust Factor " + trustFactor + " -> " + ((1 - trustFactor) * estimateUncertainty));
        trustFactor = estimateUncertainty / (estimateUncertainty + sensorUncertainty);

        // Update the state of the filter using the raw reading
        stateEstimate = stateEstimate + trustFactor * (rawSensorReading - stateEstimate);

        // Update the covariance guess
        Log.d("Kalman", "Second Estimate uncertainty " + estimateUncertainty + " -> " + ((1 - trustFactor) * estimateUncertainty));
        estimateUncertainty = (1 - trustFactor) * estimateUncertainty;

        // Update the previous state and covariance guess
        previousStateEstimate = stateEstimate;
        previousEstimateUncertainty = estimateUncertainty;

        // Update the previous measurement
        previousMeasurement = rawSensorReading;
    }

    /**
     * Returns the current estimate of the system's state, as calculated by the
     * kalman filter.
     *
     * @return The current estimate of the system's state.
     */
    public double getStateEstimate() {
        return stateEstimate;
    }

    /**
     * Returns the current uncertainty of the filter's state estimate.
     *
     * The estimate uncertainty is a measure of how confident the filter is in its estimate,
     * and is influenced by the filter's model uncertainty and sensor uncertainty.
     *
     * @return The current uncertainty of the filter's state estimate.
     */
    public double getEstimateUncertainty() {
        return estimateUncertainty;
    }

    /**
     * Returns the current trust factor for the filter's state estimate.
     *
     * The trust factor is a measure of how much the filter trusts the current
     * sensor measurements, relative to its previous estimate of the system's state. A high
     * trust factor indicates that the filter trusts the current sensor measurements more than
     * its previous estimate, and a low trust factor indicates the opposite.
     *
     * @return The current trust factor for the filter's state estimate.
     */
    public double getTrustFactor() {
        return trustFactor;
    }
}
