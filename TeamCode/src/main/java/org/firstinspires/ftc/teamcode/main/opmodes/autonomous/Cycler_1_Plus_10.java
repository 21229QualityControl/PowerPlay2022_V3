package org.firstinspires.ftc.teamcode.main.opmodes.autonomous;

import android.util.Log;

import org.firstinspires.ftc.teamcode.main.subsystems.Intake;
import org.firstinspires.ftc.teamcode.main.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.function.Supplier;

public class Cycler_1_Plus_10 {
    private Roadrunner roadrunner;
    private Intake intake;
    private Outtake outtake;

    public Cycler_1_Plus_10(Roadrunner rd, Intake intk, Outtake outk) {
        this.roadrunner = rd;
        this.intake = intk;
        this.outtake = outk;
    }

    public TrajectorySequence cycle(TrajectorySequenceBuilder builder, double turretAngle, int stackLayer, Supplier<Double> runtimeSupplier) {
        return builder
                .setKeepPosition(true)
                .executeSync(() -> {
                    // move intake out
                    intake.extenderTo(Intake.EXTENDER_BEFORE_STACK_POS + 40);
                    intake.vslideLevel(stackLayer);
                    intake.armIntake();
                    intake.clawRelease(); // to go through gap

                    waitSeconds(0.01);
                    if (Thread.interrupted()) return;

                    // reset slide encoder
                    if (outtake.isSlideMagnetPresent()) {
                        Log.d("Outtake", "Reset slide encoders from " + outtake.getSlidePosition());
                        outtake.getSlide().zeroMotorInternals();
                    } else {
                        Log.d("Outtake", "Skipped slide encoder reset due to missing magnet");
                    }

                    // raise up
                    outtake.armTiltOut();
                    outtake.guideFlatOut();
                    outtake.raiseHigh();
                    outtake.setTurretAngle(turretAngle);

                    waitSeconds(0.1);
                    if (Thread.interrupted()) return;

                    // latch cone
                    outtake.latchBarely();

                    waitSeconds(0.3);
                    if (Thread.interrupted()) return;

                    // drop cone
                    outtake.latchOpen();
                    outtake.guideRetractDown();

                    waitSeconds(0.15);
                    if (Thread.interrupted()) return;

                    // extend intake final while cone is dropping
                    intake.extendCycle();

                    waitSeconds(0.1);
                    if (Thread.interrupted()) return;

                    // raise down
                    outtake.store();
                    outtake.turretCenter();
                    outtake.guideStoreUp();
                    outtake.armTransfer();

                    // grab next
                    intake.clawGrab();

                    waitSeconds(0.2);
                    if (Thread.interrupted()) return;

                    // lift off stack
                    intake.vslideLiftLevel(stackLayer);

                    waitSeconds(0.1);
                    if (Thread.interrupted()) return;

                    // start arm
                    intake.armTransferAuto();
                    if (stackLayer != 5) {
                        waitSeconds(0.15);
                    } else { // top layer receives special treatment
                        waitSeconds(0.10);
                        intake.extenderTo(intake.getExtenderTarget() - 80);
                        waitSeconds(0.15); // wait extra for top layer
                    }
                    if (Thread.interrupted()) return;

                    // pull back soon after
                    intake.extenderTo(60); // TODO: Put this into the constant

                    waitSeconds(0.1);
                    if (Thread.interrupted()) return;

                    // lower for transfer soon after
                    intake.armTransferAuto();
                    intake.vslideTransferAuto();

                    waitSeconds(0.2);
                    if (Thread.interrupted()) return;

                    // drop cone onto holder
                    intake.clawRelease();

                    waitSeconds(0.3);
                })
                .build();
    }

    public TrajectorySequence sendLastCone(TrajectorySequenceBuilder builder, double turretAngle) {
        return builder
                .setKeepPosition(true)
                .executeSync(() -> {
                    // move intake out of the way
                    intake.armStore();
                    intake.clawRelease(); // to go through gap

                    waitSeconds(0.01);
                    if (Thread.interrupted()) return;

                    // reset slide encoder
                    if (outtake.isSlideMagnetPresent()) {
                        Log.d("Outtake", "Reset slide encoders from " + outtake.getSlidePosition());
                        outtake.getSlide().zeroMotorInternals();
                    } else {
                        Log.d("Outtake", "Skipped slide encoder reset due to missing magnet");
                    }

                    // raise up
                    outtake.armTiltOut();
                    outtake.guideFlatOut();
                    outtake.raiseHigh();
                    outtake.setTurretAngle(turretAngle);

                    waitSeconds(0.1);
                    if (Thread.interrupted()) return;

                    // latch cone
                    outtake.latchBarely();

                    waitSeconds(0.3);
                    if (Thread.interrupted()) return;

                    // drop cone
                    outtake.latchOpen();
                    outtake.guideRetractDown();

                    waitSeconds(0.25);
                    if (Thread.interrupted()) return;

                    // raise down
                    outtake.store();
                    outtake.turretCenter();
                    outtake.guideStoreUp();
                    outtake.armTransfer();

                    waitSeconds(0.4);
                })
                .build();
    }

    private void waitSeconds(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Preserve interrupted status
        }
    }
}
