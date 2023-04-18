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
    public static double TRANSFER_INTAKE_ARM = Intake.ARM_TRANSFER_AUTO_POS;
    public static int TRANSFER_INTAKE_EXTENSION = Intake.EXTENDER_TRANSFER_AUTO_POS;
    public static double TRANSFER_INTAKE_VSLIDE = Intake.VSLIDE_TRANSFER_AUTO_POS;
    public static double TRANSFER_OUTTAKE_ARM = Outtake.ARM_TRANSFER_AUTO_POS;

    public static int TESTING_INTAKE_EXTENSION = Intake.EXTENDER_CYCLE_POS;
    public static double TESTING_INTAKE_ARM = Intake.ARM_INTAKING_POS;
    public static double TESTING_INTAKE_VSLIDE = Intake.VSLIDE_LVL1_POS;

    private Intake intake;
    private Outtake outtake;

    @Override
    public void runOpMode() throws InterruptedException {

        // Communicate to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create our subsystems
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        intake.getExtender().zeroMotorInternals();
        outtake.getSlide().zeroMotorInternals();

        intake.clawRelease();
        intake.armStore();
        intake.vslideLevel(1);
        outtake.armTransferAuto();

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
                intake.extenderTo(TESTING_INTAKE_EXTENSION);
                intake.setArmPosition(TESTING_INTAKE_ARM);
                intake.setVSlidePosition(TESTING_INTAKE_VSLIDE);
                intake.clawRelease();
                outtake.setArmPosition(TRANSFER_OUTTAKE_ARM);
                outtake.latchOpen();
            }

            if (g1.aOnce()) { // grab or release
                if (!intake.isClawClosed()) intake.clawGrab();
                else intake.clawRelease();
            }

            // Transfer
            if (g1.xOnce()) {
                outtake.setArmPosition(TRANSFER_OUTTAKE_ARM);
                intake.extenderTo(TRANSFER_INTAKE_EXTENSION);
                intake.setArmPosition(TRANSFER_INTAKE_ARM);
                intake.setVSlidePosition(TRANSFER_INTAKE_VSLIDE);
            }

            // Drop
            if (g1.yOnce()) {
                intake.clawRelease();
            }

            // Update extender PID
            intake.update();
        }
    }
}
