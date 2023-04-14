package org.firstinspires.ftc.teamcode.main.opmodes.autonomous.safe_1_plus_5;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.main.environment.FieldConstants;
import org.firstinspires.ftc.teamcode.main.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StateCopyLocalizer;
import org.firstinspires.ftc.teamcode.util.data.CPose2d;

@Config
@Autonomous(name = "🔴 ◢ Red Right Auto Safe 1+5", group = "Auto Safe 1+5", preselectTeleOp = "Manual Drive")
public class AutoRedRight_Safe_1_Plus_5_Park extends AutoBase {
    public static CPose2d SCORING_POSITION = new CPose2d(FieldConstants.redRightConeStack.plus(new Vector2d(-58, 0)), Math.toRadians(0));
    public static CPose2d INTAKING_POSITION = new CPose2d(FieldConstants.redRightConeStack.plus(new Vector2d(-46, 0)), Math.toRadians(0));
    public static double TURRET_ANGLE = 49;
    public static int EXTENDER_TICKS = 1050;

    @Override
    protected boolean isBlue() {
        return false;
    }

    @Override
    protected Pose2d getStartPose() {
        return FieldConstants.redRightStartingPosition;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "🔴⌊    ◢⌋ Shared 1+5 and park");
    }

    @Override
    protected void onRun() {
        Log.d("Autonomous", String.format("onRun() Start %.3f", getRuntime()));

        moveOver();

        for (int stack = 5; stack > 0; stack--) {
            moveToJunctionAndScore(stack);
            moveToStackAndIntake(stack);
        }

        moveToJunctionAndScore(0);

        park();
    }

    private void moveOver() {
        Log.d("Autonomous", String.format("moveOver() Start %.3f", getRuntime()));
        follow(builder()
                .lineTo(new Vector2d(getStartPose().getX(), SCORING_POSITION.getY()))
                .turnTo(SCORING_POSITION.getHeading())
                .build());

        Log.d("Autonomous", String.format("moveOver() Ended %.3f", getRuntime()));
    }

    private void moveToJunctionAndScore(int nextLayerStack) {
        follow(builder(new Pose2d(StateCopyLocalizer.pose.vec(), INTAKING_POSITION.getHeading()))
                .setKeepPosition(true)
                .lineToLinearHeading(SCORING_POSITION.asPose2d())
                .addTemporalMarker(0, 2, () -> {
                    if (nextLayerStack > 0) {
                        intake.extenderTo(EXTENDER_TICKS);
                        intake.vslideLevel(nextLayerStack);
                        intake.armIntake();
                        intake.clawRelease(); // to go through gap
                    }
                })

                .addTemporalMarker(1, -0.5, () -> {
                    // raise up
                    outtake.armTiltOut();
                    outtake.guideFlatOut();
                    outtake.raiseHigh();
                    outtake.setTurretAngle(TURRET_ANGLE);
                })
                .addTemporalMarker(1, -0.4, () -> {
                    outtake.latchBarely();
                })
                .executeSync(() -> {
                    // drop cone
                    outtake.latchOpen();
                    outtake.guideRetractDown();

                    waitSecondsSimple(0.25);
                    if (Thread.interrupted()) return;

                    // raise down
                    outtake.store();
                    outtake.turretCenter();
                    outtake.guideStoreUp();
                    outtake.armTransfer();
                    waitSecondsSimple(0.3);
                })
                .build());
    }

    private void moveToStackAndIntake(int stackLayer) {
        follow(builder(SCORING_POSITION.asPose2d())
                .lineToLinearHeading(INTAKING_POSITION.asPose2d())
                .addTemporalMarker(1, -0.3, () -> {
                    intake.clawGrab();
                })
                .executeSync(() -> {
                    // lift off stack
                    intake.vslideLiftLevel(stackLayer);

                    waitSecondsSimple(0.3);
                    if (Thread.interrupted()) return;

                    // start arm
                    intake.armStore();

                    if (stackLayer != 5) {
                        waitSecondsSimple(0.20);
                    } else { // top layer receives special treatment
                        waitSecondsSimple(0.10);
                        intake.extenderTo(intake.getExtenderTarget() - 80);
                        waitSecondsSimple(0.30); // wait extra for top layer
                    }
                    if (Thread.interrupted()) return;

                    // pull back soon after
                    intake.extendTransferAuto();

                    waitSecondsSimple(0.2);
                    if (Thread.interrupted()) return;

                    intake.armTransferAuto();
                    intake.vslideTransferAuto();

                    waitSecondsSimple(0.4);
                    if (Thread.interrupted()) return;

                    // drop cone onto holder
                    intake.clawRelease();

                    waitSecondsSimple(0.1);
                    if (Thread.interrupted()) return;

                    // move claw completely out of the way
                    intake.armIntake();

                    waitSecondsSimple(0.01);
                })
                .build());
    }

    private void park() {
        Log.d("Autonomous", String.format("park() Start %.3f", getRuntime()));
        outtake.setTurretAngle(0);
        outtake.store();
        intake.armStore();
        intake.extendStore();
        intake.clawClosed();
        intake.vslideDown();
        switch (SIGNAL) {
            case 1:
                follow(builder()
                        .strafeTo(FieldConstants.getSquareCenter(2, 3).plus(new Vector2d(0.5)))
                        .build());
                break;
            default:
                follow(builder()
                        .strafeTo(FieldConstants.getSquareCenter(1, 3).plus(new Vector2d(0.5)))
                        .build());
                break;
            case 3:
                follow(builder()
                        .strafeTo(FieldConstants.getSquareCenter(0, 3).plus(new Vector2d(2)))
                        .build());
                break;
        }
        Log.d("Autonomous", String.format("park() Ended %.3f", getRuntime()));
    }

    private void waitSecondsSimple(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Preserve interrupted status
        }
    }
}
