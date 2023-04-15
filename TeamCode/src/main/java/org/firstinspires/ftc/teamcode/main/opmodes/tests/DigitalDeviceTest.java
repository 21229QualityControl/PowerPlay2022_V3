package org.firstinspires.ftc.teamcode.main.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.main.subsystems.GamePadController;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * This is a test program for testing any digital device's reading.
 */
@Config
//@Disabled
@TeleOp(group = "Test")
public class DigitalDeviceTest extends LinearOpMode {
    public static String NAME = "digitalDevice";
    public static DigitalChannel.Mode MODE = DigitalChannel.Mode.INPUT;

    private List<String> digitalDeviceNames;
    private static int digitalDeviceIndex = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        // Communicate to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Pull a list of all the servos from the configuration
        digitalDeviceNames = new ArrayList<>(hardwareMap.digitalChannel.size());
        hardwareMap.digitalChannel.entrySet().forEach(entry -> digitalDeviceNames.add(entry.getKey()));
        Collections.sort(digitalDeviceNames);
        if (!digitalDeviceNames.contains(NAME)) digitalDeviceIndex = -1; // reset index if dashboard changed it

        // Create our controller interface
        GamePadController g1 = new GamePadController(gamepad1);

        // Get input
        while (opModeInInit()) {
            // Control digital device selection
            g1.update();
            if (g1.leftBumperOnce()) {
                digitalDeviceIndex = Range.clip(digitalDeviceIndex - 1, 0, digitalDeviceNames.size() - 1);
                NAME = digitalDeviceNames.get(digitalDeviceIndex);
            }
            if (g1.rightBumperOnce()) {
                digitalDeviceIndex = Range.clip(digitalDeviceIndex + 1, 0, digitalDeviceNames.size() - 1);
                NAME = digitalDeviceNames.get(digitalDeviceIndex);
            }
            if (g1.aOnce()) {
                if (MODE == DigitalChannel.Mode.INPUT) MODE = DigitalChannel.Mode.OUTPUT;
                else MODE = DigitalChannel.Mode.INPUT;
            }

            // Telemetry state and instructions
            telemetry.addLine("~ Use bumpers to scroll through digital device list");
            telemetry.addLine("~ Use A to swap mode (You usually don't need to)");
            telemetry.addLine();
            if (!digitalDeviceNames.contains(NAME)) telemetry.addLine("*** THIS DIGITAL DEVICE DOES NOT EXIST ***");
            telemetry.addData("Digital device name", NAME);
            telemetry.addData("Digital device mode", MODE);
            telemetry.update();
        }

        // Exit immediately if stopped
        if (isStopRequested()) return;

        // Create the digital device
        DigitalChannel digitalDevice = hardwareMap.digitalChannel.get(NAME);

        // Set the mode.
        digitalDevice.setMode(MODE);

        // Game loop
        while (!isStopRequested()) {
            // Telemetry state
            telemetry.addData("State", digitalDevice.getState());
            telemetry.addData("Digital device name", NAME);
            telemetry.addData("Digital device mode", MODE);
            telemetry.update();
        }
    }
}