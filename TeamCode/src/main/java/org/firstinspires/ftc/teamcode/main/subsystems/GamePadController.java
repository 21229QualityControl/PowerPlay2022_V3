package org.firstinspires.ftc.teamcode.main.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Class that simplifies controller logic.
 * Offers press, hold, release, and more.
 */
public class GamePadController {
    private final double triggerThreshold = 0.8;
    private final double joystickThreshold = 0.75; // Warning, X box controller rightStick +Y seems to max out at ~0.8, probably broken

    private boolean active = true;

    private Gamepad gamepad;

    private PressDuration dpad_up = new PressDuration(), dpad_down = new PressDuration(), dpad_left = new PressDuration(), dpad_right = new PressDuration();
    private PressDuration x = new PressDuration(), y = new PressDuration(), a = new PressDuration(), b = new PressDuration();
    private PressDuration left_bumper = new PressDuration(), right_bumper = new PressDuration();
    private PressDuration start = new PressDuration(), back = new PressDuration(), guide = new PressDuration();
    private PressDuration left_trigger_it = new PressDuration(), right_trigger_it = new PressDuration();
    private PressDuration left_stick_button = new PressDuration(), right_stick_button = new PressDuration();
    private PressDuration left_stick_px_it = new PressDuration(), left_stick_nx_it = new PressDuration(), left_stick_py_it = new PressDuration(), left_stick_ny_it = new PressDuration();
    private PressDuration right_stick_px_it = new PressDuration(), right_stick_nx_it = new PressDuration(), right_stick_py_it = new PressDuration(), right_stick_ny_it = new PressDuration();

    public double left_stick_x = 0;
    public double right_stick_x = 0;
    public double left_stick_y = 0;
    public double right_stick_y = 0;

    public double left_trigger = 0;
    public double right_trigger = 0;

    public GamePadController(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void enable() {
        active = true;
    }
    public void disable() {
        active = false;
        update(); // update so everything turns to 0
    }
    public boolean isActive() {
        return active;
    }

    public boolean isConnected() {
        return gamepad.getGamepadId() > 0;
    }
    public boolean isLostConnection() {
        return gamepad.getGamepadId() == Gamepad.ID_SYNTHETIC;
    }
    public boolean isNotPaired() { // unreliable, a connected controller may not be detected until it inputs some action
        return gamepad.getGamepadId() == Gamepad.ID_UNASSOCIATED;
    }

    /**
     * Counts how many compute cycles something has been pressed
     */
    public static class PressDuration {
        private int n = 0;

        private void update(boolean isPressed) {
            if (isPressed) press();
            else release();
        }
        private void press() {
            n = n<0? 1 : n+1;
        }
        private void release() {
            n = n>0? -1 : 0;
        }
    }

    public void update() {
        // Buttons
        x.update(gamepad.x && active);
        y.update(gamepad.y && active);
        a.update(gamepad.a && active);
        b.update(gamepad.b && active);
        dpad_up.update(gamepad.dpad_up && active);
        dpad_down.update(gamepad.dpad_down && active);
        dpad_left.update(gamepad.dpad_left && active);
        dpad_right.update(gamepad.dpad_right && active);
        left_bumper.update(gamepad.left_bumper && active);
        right_bumper.update(gamepad.right_bumper && active);
        start.update(gamepad.start && active);
        back.update(gamepad.back && active);
        guide.update(gamepad.guide && active);
        left_stick_button.update(gamepad.left_stick_button && active);
        right_stick_button.update(gamepad.right_stick_button && active);

        // Triggers
        left_trigger_it.update(gamepad.left_trigger > triggerThreshold && active);
        right_trigger_it.update(gamepad.right_trigger > triggerThreshold && active);

        // Left Stick
        left_stick_nx_it.update(gamepad.left_stick_x < -joystickThreshold && active);
        left_stick_px_it.update(gamepad.left_stick_x > joystickThreshold && active);
        left_stick_ny_it.update(gamepad.left_stick_y < -joystickThreshold && active);
        left_stick_py_it.update(gamepad.left_stick_y > joystickThreshold && active);

        // Right Stick
        right_stick_nx_it.update(gamepad.right_stick_x < -joystickThreshold && active);
        right_stick_px_it.update(gamepad.right_stick_x > joystickThreshold && active);
        right_stick_ny_it.update(gamepad.right_stick_y < -joystickThreshold && active);
        right_stick_py_it.update(gamepad.right_stick_y > joystickThreshold && active);

        // Variables
        if (active) { left_stick_x = gamepad.left_stick_x; } else { left_stick_x = 0; }
        if (active) { left_stick_y = gamepad.left_stick_y; } else { left_stick_y = 0; }
        if (active) { right_stick_x = gamepad.right_stick_x; } else { right_stick_x = 0; }
        if (active) { right_stick_y = gamepad.right_stick_y; } else { right_stick_y = 0; }
        if (active) { left_trigger = gamepad.left_trigger; } else { left_trigger = 0; }
        if (active) { right_trigger = gamepad.right_trigger; } else { right_trigger = 0; }
    }

    // isDown
    public boolean dpadUp() { return 0 < dpad_up.n; }
    public boolean dpadDown() { return 0 < dpad_down.n; }
    public boolean dpadLeft() { return 0 < dpad_left.n; }
    public boolean dpadRight() { return 0 < dpad_right.n; }
    public boolean x() { return 0 < x.n; }
    public boolean y() { return 0 < y.n; }
    public boolean a() { return 0 < a.n && !start(); }
    public boolean b() { return 0 < b.n && !start(); }
    public boolean leftBumper() { return 0 < left_bumper.n; }
    public boolean rightBumper() { return 0 < right_bumper.n; }
    public boolean leftTrigger() { return 0 < left_trigger_it.n;}
    public boolean rightTrigger() { return 0 < right_trigger_it.n;}
    public boolean leftStickButton() { return 0 < left_stick_button.n; }
    public boolean rightStickButton() { return 0 < right_stick_button.n; }
    public boolean leftStickRight() { return 0 < left_stick_px_it.n; }
    public boolean leftStickUp() { return 0 < left_stick_ny_it.n; }
    public boolean leftStickLeft() { return 0 < left_stick_nx_it.n; }
    public boolean leftStickDown() { return 0 < left_stick_py_it.n; }
    public boolean rightStickRight() { return 0 < right_stick_px_it.n; }
    public boolean rightStickUp() { return 0 < right_stick_ny_it.n; }
    public boolean rightStickLeft() { return 0 < right_stick_nx_it.n; }
    public boolean rightStickDown() { return 0 < right_stick_py_it.n; }
    public boolean start() {return 0 < start.n;}
    public boolean back() {return 0 < back.n;}
    public boolean guide() {return 0 < guide.n;}

    // isPressed
    public boolean dpadUpOnce() { return 1 == dpad_up.n; }
    public boolean dpadDownOnce() { return 1 == dpad_down.n; }
    public boolean dpadLeftOnce() { return 1 == dpad_left.n; }
    public boolean dpadRightOnce() { return 1 == dpad_right.n; }
    public boolean xOnce() { return 1 == x.n; }
    public boolean yOnce() { return 1 == y.n; }
    public boolean aOnce() { return 1 == a.n && !start(); }
    public boolean bOnce() { return 1 == b.n && !start(); }
    public boolean leftBumperOnce() { return 1 == left_bumper.n; }
    public boolean rightBumperOnce() { return 1 == right_bumper.n; }
    public boolean leftTriggerOnce() { return 1 == left_trigger_it.n;}
    public boolean rightTriggerOnce() { return 1 == right_trigger_it.n;}
    public boolean leftStickButtonOnce() { return 1 == left_stick_button.n; }
    public boolean rightStickButtonOnce() { return 1 == right_stick_button.n; }
    public boolean leftStickRightOnce() { return 1 == left_stick_px_it.n; }
    public boolean leftStickUpOnce() { return 1 == left_stick_ny_it.n; }
    public boolean leftStickLeftOnce() { return 1 == left_stick_nx_it.n; }
    public boolean leftStickDownOnce() { return 1 == left_stick_py_it.n; }
    public boolean rightStickRightOnce() { return 1 == right_stick_px_it.n; }
    public boolean rightStickUpOnce() { return 1 == right_stick_ny_it.n; }
    public boolean rightStickLeftOnce() { return 1 == right_stick_nx_it.n; }
    public boolean rightStickDownOnce() { return 1 == right_stick_py_it.n; }
    public boolean startOnce() {return 1 == start.n;}
    public boolean backOnce() {return 1 == back.n;}
    public boolean guideOnce() {return 1 == guide.n;}

    // isLong
    public boolean dpadUpLong() { return 9 < dpad_up.n; }
    public boolean dpadDownLong() { return 9 < dpad_down.n; }
    public boolean dpadLeftLong() { return 9 < dpad_left.n; }
    public boolean dpadRightLong() { return 9 < dpad_right.n; }
    public boolean xLong() { return 9 < x.n; }
    public boolean yLong() { return 9 < y.n; }
    public boolean aLong() { return 9 < a.n; }
    public boolean bLong() { return 9 < b.n; }
    public boolean leftBumperLong() { return 9 < left_bumper.n; }
    public boolean rightBumperLong() { return 9 < right_bumper.n; }
    public boolean leftTriggerLong() { return 9 < left_trigger_it.n;}
    public boolean rightTriggerLong() { return 9 < right_trigger_it.n;}
    public boolean leftStickButtonLong() { return 9 < left_stick_button.n; }
    public boolean rightStickButtonLong() { return 9 < right_stick_button.n; }
    public boolean leftStickRightLong() { return 9 < left_stick_px_it.n; }
    public boolean leftStickUpLong() { return 9 < left_stick_ny_it.n; }
    public boolean leftStickLeftLong() { return 9 < left_stick_nx_it.n; }
    public boolean leftStickDownLong() { return 9 < left_stick_py_it.n; }
    public boolean rightStickRightLong() { return 9 < right_stick_px_it.n; }
    public boolean rightStickUpLong() { return 9 < right_stick_ny_it.n; }
    public boolean rightStickLeftLong() { return 9 < right_stick_nx_it.n; }
    public boolean rightStickDownLong() { return 9 < right_stick_py_it.n; }
    public boolean startLong() {return 9 < start.n;}
    public boolean backLong() {return 9 < back.n;}
    public boolean guideLong() {return 9 < guide.n;}

    // isLongPressed
    public boolean dpadUpLongOnce() { return 10 == dpad_up.n; }
    public boolean dpadDownLongOnce() { return 10 == dpad_down.n; }
    public boolean dpadLeftLongOnce() { return 10 == dpad_left.n; }
    public boolean dpadRightLongOnce() { return 10 == dpad_right.n; }
    public boolean xLongOnce() { return 10 == x.n; }
    public boolean yLongOnce() { return 10 == y.n; }
    public boolean aLongOnce() { return 10 == a.n; }
    public boolean bLongOnce() { return 10 == b.n; }
    public boolean leftBumperLongOnce() { return 10 == left_bumper.n; }
    public boolean rightBumperLongOnce() { return 10 == right_bumper.n; }
    public boolean leftTriggerLongOnce() { return 10 == left_trigger_it.n;}
    public boolean rightTriggerLongOnce() { return 10 == right_trigger_it.n;}
    public boolean leftStickButtonLongOnce() { return 10 == left_stick_button.n; }
    public boolean rightStickButtonLongOnce() { return 10 == right_stick_button.n; }
    public boolean leftStickRightLongOnce() { return 10 == left_stick_px_it.n; }
    public boolean leftStickUpLongOnce() { return 10 == left_stick_ny_it.n; }
    public boolean leftStickLeftLongOnce() { return 10 == left_stick_nx_it.n; }
    public boolean leftStickDownLongOnce() { return 10 == left_stick_py_it.n; }
    public boolean rightStickRightLongOnce() { return 10 == right_stick_px_it.n; }
    public boolean rightStickUpLongOnce() { return 10 == right_stick_ny_it.n; }
    public boolean rightStickLeftLongOnce() { return 10 == right_stick_nx_it.n; }
    public boolean rightStickDownLongOnce() { return 10 == right_stick_py_it.n; }
    public boolean startLongOnce() {return 10 == start.n;}
    public boolean backLongOnce() {return 10 == back.n;}
    public boolean guideLongOnce() {return 10 == guide.n;}

    // isReleased
    public boolean dpadUpReleased() { return -1 == dpad_up.n; }
    public boolean dpadDownReleased() { return -1 == dpad_down.n; }
    public boolean dpadLeftReleased() { return -1 == dpad_left.n; }
    public boolean dpadRightReleased() { return -1 == dpad_right.n; }
    public boolean xReleased() { return -1 == x.n; }
    public boolean yReleased() { return -1 == y.n; }
    public boolean aReleased() { return -1 == a.n && !start(); }
    public boolean bReleased() { return -1 == b.n && !start(); }
    public boolean leftBumperReleased() { return -1 == left_bumper.n; }
    public boolean rightBumperReleased() { return -1 == right_bumper.n; }
    public boolean leftTriggerReleased() { return -1 == left_trigger_it.n;}
    public boolean rightTriggerReleased() { return -1 == right_trigger_it.n;}
    public boolean leftStickButtonReleased() { return -1 == left_stick_button.n; }
    public boolean rightStickButtonReleased() { return -1 == right_stick_button.n; }
    public boolean leftStickRightReleased() { return -1 == left_stick_px_it.n; }
    public boolean leftStickUpReleased() { return -1 == left_stick_ny_it.n; }
    public boolean leftStickLeftReleased() { return -1 == left_stick_nx_it.n; }
    public boolean leftStickDownReleased() { return -1 == left_stick_py_it.n; }
    public boolean rightStickRightReleased() { return -1 == right_stick_px_it.n; }
    public boolean rightStickUpReleased() { return -1 == right_stick_ny_it.n; }
    public boolean rightStickLeftReleased() { return -1 == right_stick_nx_it.n; }
    public boolean rightStickDownReleased() { return -1 == right_stick_py_it.n; }
    public boolean startReleased() {return -1 == start.n;}
    public boolean backReleased() {return -1 == back.n;}
    public boolean guideReleased() {return -1 == guide.n;}

    public boolean atRest() {
        return !dpadUp() &&
                !dpadDown() &&
                !dpadLeft() &&
                !dpadRight() &&
                !x() &&
                !y() &&
                !a() &&
                !b() &&
                !leftBumper() &&
                !rightBumper() &&
                !leftTrigger() &&
                !rightTrigger() &&
                !leftStickButton() &&
                !rightStickButton() &&
                !start() &&
                !back() &&
                !guide() &&
                Math.abs(left_stick_x) <= 1e-1 &&
                Math.abs(left_stick_y) <= 1e-1 &&
                Math.abs(right_stick_x) <= 1e-1 &&
                Math.abs(right_stick_y) <= 1e-1 &&
                left_trigger <= 1e-1 &&
                right_trigger <= 1e-1;
    }

    public void reset() {
        disable();
        enable();
    }


    /* Rumble Support, copied from Gamepad */

    /**
     * Rumble the gamepad's first rumble motor at maximum power for a certain duration.
     * Calling this will displace any currently running rumble effect.
     * @param durationMs milliseconds to rumble for, or -1 for continuous
     */
    public void rumble(int durationMs) {
        gamepad.rumble(durationMs);
    }

    /**
     * Rumble the gamepad at a fixed rumble power for a certain duration
     * Calling this will displace any currently running rumble effect
     * @param rumble1 rumble power for rumble motor 1 (0.0 - 1.0)
     * @param rumble2 rumble power for rumble motor 2 (0.0 - 1.0)
     * @param durationMs milliseconds to rumble for, or -1 for continuous
     */
    public void rumble(double rumble1, double rumble2, int durationMs) {
        gamepad.rumble(rumble1, rumble2, durationMs);
    }

    /**
     * Rumble the gamepad for a certain number of "blips" using predetermined blip timing
     * This will displace any currently running rumble effect.
     * @param count the number of rumble blips to perform
     */
    public void rumbleBlips(int count) {
        gamepad.rumbleBlips(count);
    }

    /**
     * Run a rumble effect built using {@link Gamepad.RumbleEffect.Builder}
     * The rumble effect will be run asynchronously; your OpMode will
     * not halt execution while the effect is running.
     *
     * Calling this will displace any currently running rumble effect
     */
    public void runRumbleEffect(Gamepad.RumbleEffect effect) {
        gamepad.runRumbleEffect(effect);
    }

    /**
     * Cancel the currently running rumble effect, if any
     */
    public void stopRumble() {
        gamepad.stopRumble();
    }

    /**
     * Returns an educated guess about whether there is a rumble action ongoing on this gamepad
     * @return an educated guess about whether there is a rumble action ongoing on this gamepad
     */
    public boolean isRumbling() {
        return gamepad.isRumbling();
    }

    public Gamepad.RumbleEffect.Builder effectBuilder() {
        return new Gamepad.RumbleEffect.Builder();
    }
}