package org.firstinspires.ftc.teamcode.main.opmodes.autonomous;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.main.environment.FieldConstants;

@Config
@Autonomous(name = "🔴 ◣ Red Left Auto Simple", group = "AutoSimple", preselectTeleOp = "Manual Drive")
public class AutoSimpleRedLeft_SharedPreload_Park extends AutoBase {
    @Override
    protected boolean isBlue() {
        return true;
    }

    @Override
    protected Pose2d getStartPose() {
        return FieldConstants.redLeftStartingPosition;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "🔴⌊◣    ⌋ not implemented sample");
    }

    @Override
    protected void moveToJunction() {
        Log.d("Autonomous", String.format("step1() Start %.3f", getRuntime()));
        follow(builder()
                .executeSync(() -> {
                    // Do stuff
                })
                .build());
        Log.d("Autonomous", String.format("step1() Ended %.3f", getRuntime()));
    }

    @Override
    protected void park() {
        Log.d("Autonomous", String.format("park() Start %.3f", getRuntime()));
        // park code
        Log.d("Autonomous", String.format("park() Ended %.3f", getRuntime()));
    }
}
