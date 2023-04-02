package org.firstinspires.ftc.teamcode.main.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SmartGameTimer {
    private double offset;
    private TIMER_START_MODE mode;
    private ElapsedTime elapsedTime;

    public enum TIMER_START_MODE {
        STANDARD, // standard game timer 2:00
        FILE_RECOVERED, // based off of auto start time
        STANDARD_FALLBACK // fallback to standard when file time is too early or nonexistant
    }

    public SmartGameTimer(boolean standardOperation) {
        if (standardOperation) {
            this.offset = 0;
            this.elapsedTime = new ElapsedTime();
            mode = TIMER_START_MODE.STANDARD;
        } else {
            try {
                // get the value from memory
                String autoStartTimeStr = Memory.getStringFromFile(Memory.SAVED_TIME_FILE_NAME);
                long autoStartTime = Long.parseLong(autoStartTimeStr);
                long calculatedTeleopStartTime = autoStartTime + 38 * 1000;

                long currentTime = System.currentTimeMillis();
                offset = (currentTime - calculatedTeleopStartTime) * 0.001;

                if (offset > 120) { // if game already ended, fallback
                    this.mode = TIMER_START_MODE.STANDARD_FALLBACK;
                    this.offset = 0;
                    this.elapsedTime = new ElapsedTime();
                } else {
                    this.mode = TIMER_START_MODE.FILE_RECOVERED;
                    this.elapsedTime = new ElapsedTime();
                }
            } catch (Exception e) {
                this.mode = TIMER_START_MODE.STANDARD_FALLBACK;
                this.offset = 0;
                this.elapsedTime = new ElapsedTime();
            }
        }
    }

    public double seconds() {
        return elapsedTime.seconds() + offset;
    }

    public void reset() {
        offset = 0;
        elapsedTime.reset();
    }

    public void resetIfStandard() {
        if (mode == TIMER_START_MODE.STANDARD || mode == TIMER_START_MODE.STANDARD_FALLBACK) {
            reset();
        }
    }

    public boolean isNominal() {
        return mode == TIMER_START_MODE.STANDARD;
    }

    public boolean isRecovered() {
        return mode == TIMER_START_MODE.FILE_RECOVERED;
    }

    public boolean isLikelyWrong() {
        return mode == TIMER_START_MODE.STANDARD_FALLBACK;
    }
}
