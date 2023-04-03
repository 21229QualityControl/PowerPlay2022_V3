package org.firstinspires.ftc.teamcode.main.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Class for our Blinkin LED strip
 *
 * Memory so we don't spam logs in case of constant pattern setting
 */
public class LED {
    private RevBlinkinLedDriver ledDriver;

    BlinkinPattern pattern = null;

    public LED(HardwareMap hardwareMap) {
//        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");  // TODO: Uncomment when LED is installed
        setPattern(BlinkinPattern.BLACK);
    }

    public void setPattern(BlinkinPattern pattern) {
//        if (pattern != this.pattern) ledDriver.setPattern(pattern);  // TODO: Uncomment when LED is installed
        this.pattern = pattern;
    }

    public BlinkinPattern getPattern() {
        return pattern;
    }
}
