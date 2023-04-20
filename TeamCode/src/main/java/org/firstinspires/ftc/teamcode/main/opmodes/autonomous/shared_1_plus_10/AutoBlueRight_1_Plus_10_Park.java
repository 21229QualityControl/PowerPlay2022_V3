package org.firstinspires.ftc.teamcode.main.opmodes.autonomous.shared_1_plus_10;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.main.environment.FieldConstants;
import org.firstinspires.ftc.teamcode.main.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.main.opmodes.autonomous.Cycler_1_Plus_10;
import org.firstinspires.ftc.teamcode.util.data.CPose2d;

@Config
@Autonomous(name = "🔵 ◢ Blue Right Auto 1+10", group = "Auto 1+10", preselectTeleOp = "Manual Drive")
public class AutoBlueRight_1_Plus_10_Park extends AutoBase {
    public static CPose2d CYCLE_POSITION =        new CPose2d(FieldConstants.blueRightConeStack.plus(new Vector2d(33, 0)), Math.toRadians(180));
    public static CPose2d SECOND_CYCLE_POSITION = new CPose2d(FieldConstants.blueLeftConeStack.plus(new Vector2d(-34, 0)), Math.toRadians(0));
    public static double TURRET_ANGLE = -51;
    public static double SECOND_TURRET_ANGLE = 51;

    private Cycler_1_Plus_10 cycleManager;

    @Override
    protected boolean isBlue() {
        return true;
    }

    @Override
    protected Pose2d getStartPose() {
        return FieldConstants.blueRightStartingPosition;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "🔵⌊    ◢⌋ 1+10 and park");
    }

    @Override
    protected void onInit() {
        cycleManager = new Cycler_1_Plus_10(rr, intake, outtake);
    }

    @Override
    protected void onRun() {
        hub.setHeading(getStartPose().getHeading());
        Log.d("Angle", "IMU Heading starts " + Math.toDegrees(hub.getHeading()) + "°");
        Log.d("Autonomous", String.format("onRun() Start %.3f", getRuntime()));

        moveToJunction();

        for (int stack = 5; stack > 0; stack--) {
            Log.d("Autonomous", String.format("cycleFirstStack(" + stack + ") Start %.3f", getRuntime()));
            Log.d("Autonomous", "Started cycle at " + rr.getPoseEstimate() + ", expected " + SECOND_CYCLE_POSITION + ", error " + rr.getLastError());
            Log.d("Angle", "Raw heading is " + Math.toDegrees(hub.getHeading()) + "°");
            follow(cycleManager.cycle(builder(CYCLE_POSITION.asPose2d()), TURRET_ANGLE, stack, this::getRuntime));
            Log.d("Autonomous", String.format("cycleFirstStack(" + stack + ") Ended %.3f", getRuntime()));
        }

        moveToSecondStack();

        for (int stack = 5; stack > 0; stack--) {
            Log.d("Autonomous", String.format("cycleSecondStack(" + stack + ") Start %.3f", getRuntime()));
            Log.d("Autonomous", "Started cycle at " + rr.getPoseEstimate() + ", expected " + SECOND_CYCLE_POSITION + ", error " + rr.getLastError());
            Log.d("Angle", "Raw heading is " + Math.toDegrees(hub.getHeading()) + "°");
            follow(cycleManager.cycle(builder(SECOND_CYCLE_POSITION.asPose2d()), SECOND_TURRET_ANGLE, stack, this::getRuntime));
            Log.d("Autonomous", String.format("cycleSecondStack(" + stack + ") Ended %.3f", getRuntime()));
        }

        if (getRuntime() < 27 || (SIGNAL == 2 && getRuntime() < 28.8)) {
            Log.d("Autonomous", String.format("sendLastCone() Start %.3f", getRuntime()));
            Log.d("Angle", "Raw heading is " + Math.toDegrees(hub.getHeading()) + "°");
            follow(cycleManager.sendLastCone(builder(SECOND_CYCLE_POSITION.asPose2d()), SECOND_TURRET_ANGLE));
            Log.d("Autonomous", String.format("sendLastCone() Ended %.3f", getRuntime()));
        } else {
            Log.d("Autonomous", String.format("sendLastCone() Skipped %.3f", getRuntime()));
        }

        park();
    }

    private void moveToJunction() {
        Log.d("Autonomous", String.format("moveToJunction() Start %.3f", getRuntime()));
        follow(builder()
                .lineTo(new Vector2d(getStartPose().getX(), CYCLE_POSITION.getY()))
                .turnTo(CYCLE_POSITION.getHeading())
                .setKeepPosition(true)
                .waitSeconds(0.1)
                .waitForCondition(() -> !rr.isPositionMaintainerBusy(), 100)
                .build());

        Log.d("Autonomous", String.format("moveToJunction() Ended %.3f", getRuntime()));
    }

    private void moveToSecondStack() {
        Log.d("Autonomous", String.format("moveToSecondStack() Start %.3f", getRuntime()));
        outtake.setTurretAngle(0);
        outtake.store();
        intake.armStore();
        intake.extendStore();
        intake.clawClosed();
        intake.vslideLevel(3); // in case there's something in the intake
        follow(builder(CYCLE_POSITION.asPose2d())
                .lineTo(new Vector2d(SECOND_CYCLE_POSITION.getX(), SECOND_CYCLE_POSITION.getY()))
                .turnTo(SECOND_CYCLE_POSITION.getHeading())
                .setKeepPosition(true)
                .waitSeconds(0.1)
                .waitForCondition(() -> !rr.isPositionMaintainerBusy(), 100)
                .build());

        Log.d("Autonomous", String.format("moveToSecondStack() Ended %.3f", getRuntime()));
    }

    private void park() {
        Log.d("Autonomous", String.format("park() Start %.3f", getRuntime()));
        outtake.setTurretAngle(0);
        outtake.store();
        intake.armPark();
        intake.extendStore();
        intake.clawClosed();
        intake.vslideLevel(3); // in case there's something in the intake
        switch (SIGNAL) {
            case 1:
                follow(builder()
                        .strafeTo(FieldConstants.getSquareCenter(0, 2).plus(new Vector2d(2)))
                        .build());
                break;
            default:
                follow(builder()
                        .addIterative("Emergency brake", () -> {
                            if (getRuntime() > 29.5) rr.forceStopTrajectory();
                        })
                        .strafeTo(FieldConstants.getSquareCenter(1, 2).plus(new Vector2d(0.5)))
                        .build());
                break;
            case 3:
                follow(builder()
                        .strafeTo(FieldConstants.getSquareCenter(2, 2).plus(new Vector2d(0.5)))
                        .build());
                break;
        }
        Log.d("Autonomous", String.format("park() Ended %.3f", getRuntime()));
    }
}
