package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class that represents an axon servo and its analog encoder.
 */
public class AxonServo {
    private Servo servo;
    private AnalogInput encoder;
    private double setPosition;

    public AxonServo(HardwareMap hardwareMap, String servoName, String encoderName) {
        servo = HardwareCreator.createServo(hardwareMap, servoName, HardwareCreator.ServoType.AXON);
        encoder = hardwareMap.analogInput.get(encoderName);
        setPosition = 0;
    }

    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }

    public void setPosition(double pos) {
        servo.setPosition(pos);
        setPosition = pos;
    }

    public double getEncoderPosition() {
        return encoder.getVoltage() / 3.3;
    }

    public double getTargetPosition() {
        return setPosition;
    }
}
