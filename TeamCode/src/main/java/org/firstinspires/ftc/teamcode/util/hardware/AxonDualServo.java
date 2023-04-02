package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class that represents two connected axon servos and one of their analog encoders.
 *
 *
 * Assumes that the two servos are axon, and are linked properly (eg. one servo reversed, servo range aligned)
 */
public class AxonDualServo {
    private Servo servo1, servo2;
    private AnalogInput encoder;
    private double setPosition;
    private double servo2PositionOffset;

    public AxonDualServo(HardwareMap hardwareMap, String name1, String name2, String encoderName) {
        servo1 = HardwareCreator.createServo(hardwareMap, name1, HardwareCreator.ServoType.AXON);
        servo2 = HardwareCreator.createServo(hardwareMap, name2, HardwareCreator.ServoType.AXON);
        encoder = hardwareMap.analogInput.get(encoderName);
        setPosition = 0;
        servo2PositionOffset = 0;
    }

    public void scaleRange(double min, double max) {
        servo1.scaleRange(min, max);
        servo2.scaleRange(min, max);
    }

    public void setPosition(double pos) {
        servo1.setPosition(pos);
        servo2.setPosition(pos + servo2PositionOffset);
        setPosition = pos;
    }

    public void setServosPositionOffset(double offset) {
        servo2PositionOffset = offset;
    }

    public double getEncoderPosition() {
        return encoder.getVoltage() / 3.3;
    }

    public double getTargetPosition() {
        return setPosition;
    }
}
