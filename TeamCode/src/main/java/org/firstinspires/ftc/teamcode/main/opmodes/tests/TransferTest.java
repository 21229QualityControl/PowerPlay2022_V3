package org.firstinspires.ftc.teamcode.main.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.main.subsystems.GamePadController;
import org.firstinspires.ftc.teamcode.main.subsystems.Intake;
import org.firstinspires.ftc.teamcode.main.subsystems.Outtake;

/**
 * This teleop tests the positioning and sequencing of transfer
 */
@Config
//@Disabled
@TeleOp(group = "Test")
public class TransferTest extends LinearOpMode {

    Intake intake;
    Outtake outtake;

    @Override
    public void runOpMode() throws InterruptedException {

        // Communicate to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create our subsystems
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        intake.getExtender().zeroMotorInternals();
        outtake.getSlide().zeroMotorInternals();

        // Create our controller interface
        GamePadController g1 = new GamePadController(gamepad1);

        waitForStart();

        // Exit immediately if stopped
        if (isStopRequested()) return;

        // Game loop
        while (!isStopRequested()) {
            // Update controller
            g1.update();

            if (g1.bOnce()) { // reset
                intake.extenderTo(200);
                intake.armIntake();
                intake.vslideDown();
                intake.clawRelease();
                outtake.armTransferAuto();
                outtake.latchOpen();
            }

            if (g1.aOnce()) { // grab or release
                if (!intake.isClawClosed()) intake.clawGrab();
                else intake.clawRelease();
            }

            // Move back
            if (g1.xOnce()) {
                intake.extendTransferAuto();
                intake.vslideTransferAuto();
            }

            // Transfer
            if (g1.yOnce()) {
                intake.armTransferAuto();
            }

            // Update extender PID
            intake.update();
        }
    }
}
