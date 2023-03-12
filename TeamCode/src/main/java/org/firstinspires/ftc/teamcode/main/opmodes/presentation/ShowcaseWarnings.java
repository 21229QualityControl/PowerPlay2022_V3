package org.firstinspires.ftc.teamcode.main.opmodes.presentation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.main.subsystems.GamePadController;

/**
 * Showcase ability to notify driver with vibrations at specific times
 */
@Disabled
@TeleOp(name = "Time Warning Showcase", group = "Showcase")
public class ShowcaseWarnings extends LinearOpMode {

    private boolean warning1 = false;
    private boolean warning2 = false;
    private boolean warning3 = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamePadController g1 = new GamePadController(gamepad1);
        GamePadController g2 = new GamePadController(gamepad2);

        waitForStart();

        g1.update();
        g2.update();
        while (g1.atRest() && g2.atRest() && opModeIsActive()) {
            g1.update();
            g2.update();
            telemetry.addLine("Ready to start");
            telemetry.update();
        }

        resetRuntime();

        while (opModeIsActive()) {
            if (isBetween(timeLeft(), 0, 2)) { // last 2 sec, go park
                if (!warning3) {
                    g1.rumbleBlips(1);
                    warning3 = true;
                }
//                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            } else if (isBetween(timeLeft(), 4, 6)) { // 6-4; endgame starts
                if (!warning2) {
                    g1.rumbleBlips(2);
                    warning2 = true;
                }
//                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
            } else if (isBetween(timeLeft(), 8, 10)) { // 10-8; start capping
                if (!warning1) {
                    g1.rumbleBlips(3);
                    warning1 = true;
                }
//                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            } else { // turn off led otherwise
//                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }

            telemetry.addData("Time Left", timeLeft());
            if (!warning1) telemetry.addLine("Warning 1 triggers [10-8]");
            else if (!warning2) telemetry.addLine("Warning 2 triggers [6-4]");
            else if (!warning3) telemetry.addLine("Warning 3 triggers [2-0]");
            else telemetry.addLine("All warnings have triggered");
            telemetry.update();
        }
    }

    private double timeLeft() {
        if (!isStarted()) return 15;
        return 15 - getRuntime();
    }
    private boolean isBetween(double number, double min, double max) {
        return min <= number && number < max;
    }
}
