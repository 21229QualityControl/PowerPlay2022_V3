package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.main.subsystems.Dashboard.DISABLE_DASHBOARD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

@TeleOp(name = "Enable/Disable Dashboard Communication", group = "dash")
public class ToggleDashboardCommunication extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean enabled = !DISABLE_DASHBOARD;

        telemetry.log().add(Misc.formatInvariant("Dashboard code is currently %s. Press Start to %s communication. (May cause gc crash)",
                enabled ? "enabled" : "disabled", enabled ? "remove" : "allow"));
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        if (enabled) {
            DISABLE_DASHBOARD = true;
        } else {
            DISABLE_DASHBOARD = false;
        }
    }
}
