package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;


public class ButtonPressDetector {


    public enum Button {a,b,x,y,
        dpad_up,dpad_down,dpad_left,dpad_right,
        dpad_up_left, dpad_up_right, dpad_down_left, dpad_down_right,
        left_bumper, right_bumper, start, share,back,
        left_stick_button, right_stick_button,
        left_trigger, right_trigger,
        left_stick_left, left_stick_right, left_stick_up, left_stick_down,
        right_stick_left, right_stick_right, right_stick_up, right_stick_down,
    }
    boolean[] lastPressed = new boolean[Button.values().length];
    boolean[] lastReleased = new boolean[Button.values().length];
    boolean[] lastToggleState = new boolean[Button.values().length];

    Gamepad gamepad = null;
    double triggerThreshold=0.3;

    public ButtonPressDetector(Gamepad gamepadIn) {
        for (int i = 0; i< lastPressed.length; i++ ) {
            lastPressed[i]=false;
            lastReleased[i]=true;
            lastToggleState[i]=false;
        }
        gamepad=gamepadIn;
    }

    public void setTriggerThreshold(double thresholdIn) {
        triggerThreshold = thresholdIn;
    }

    public boolean wasToggled(Button b) {
        boolean lastState = lastToggleState[b.ordinal()];
        boolean pressed = getState(b);
        if (pressed != lastState) {
            lastToggleState[b.ordinal()] = pressed;
            return true;
        }
        return false;
    }

    public boolean wasPressed(Button b) {
        boolean lastState = lastPressed[b.ordinal()];
        boolean pressed = getState(b);
        lastPressed[b.ordinal()] = pressed;

        //If I am pressed, make sure I detect the next release.
        //But if I am released, do not remember the state changed because this is not wasReleased;
        if (pressed == true) {
            lastReleased[b.ordinal()] = false;
        }
        //System.out.println("wasPressed" + pressed + ":" + lastState);
        return (pressed == true && lastState == false);
    }

    public boolean wasReleased(Button b) {
        boolean lastPressed = !lastReleased[b.ordinal()];
        boolean pressed = getState(b);
        lastReleased[b.ordinal()] = !pressed;

        //If I am released, make sure I detect the next press.
        //But if I am pressed, do not remember because this is not wasPressed

        if (pressed == false) {
            this.lastPressed[b.ordinal()] = false;
        }

        return (pressed == false && lastPressed == true);
    }


    private boolean getState(Button b) {
        boolean thisState = false;
        switch (b) {
            case a: thisState = gamepad.a; break;
            case b: thisState = gamepad.b;break;
            case x: thisState = gamepad.x; break;
            case y: thisState = gamepad.y; break;
            case dpad_up: thisState = gamepad.dpad_up && !gamepad.dpad_left && !gamepad.dpad_right; break;
            case dpad_down: thisState = gamepad.dpad_down && !gamepad.dpad_right && !gamepad.dpad_left; break;
            case dpad_left: thisState = gamepad.dpad_left && !gamepad.dpad_up && !gamepad.dpad_down; break;
            case dpad_right: thisState = gamepad.dpad_right && !gamepad.dpad_up && !gamepad.dpad_down; break;
            case dpad_up_left: thisState = gamepad.dpad_up && gamepad.dpad_left; break;
            case dpad_up_right: thisState = gamepad.dpad_up && gamepad.dpad_right; break;
            case dpad_down_left: thisState = gamepad.dpad_down && gamepad.dpad_left; break;
            case dpad_down_right: thisState = gamepad.dpad_down && gamepad.dpad_right; break;
            case left_bumper: thisState = gamepad.left_bumper; break;
            case right_bumper: thisState = gamepad.right_bumper; break;
            case start: thisState = gamepad.start; break;
            case back: thisState = gamepad.back; break;
            case share: thisState = gamepad.share; break;
            case left_stick_button: thisState = gamepad.left_stick_button; break;
            case right_stick_button: thisState = gamepad.right_stick_button; break;
            case left_trigger: thisState = gamepad.left_trigger>triggerThreshold; break;
            case right_trigger: thisState = gamepad.right_trigger>triggerThreshold; break;
            case left_stick_left: thisState = gamepad.left_stick_x<-triggerThreshold; break;
            case left_stick_right: thisState = gamepad.left_stick_x>triggerThreshold; break;
            case left_stick_up: thisState = gamepad.left_stick_y>triggerThreshold; break;
            case left_stick_down: thisState = gamepad.left_stick_y<-triggerThreshold; break;
            case right_stick_left: thisState = gamepad.right_stick_x<-triggerThreshold; break;
            case right_stick_right: thisState = gamepad.right_stick_x>triggerThreshold; break;
            case right_stick_up: thisState = gamepad.right_stick_y>triggerThreshold; break;
            case right_stick_down: thisState = gamepad.right_stick_y<-triggerThreshold; break;
        }
        return thisState;
    }
}
