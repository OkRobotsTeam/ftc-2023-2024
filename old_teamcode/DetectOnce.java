package org.firstinspires.ftc.old_teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DetectOnce {
    public enum Button {a,b,x,y,dpad_up,dpad_down,dpad_left,dpad_right,left_bumper,
        right_bumper, start, share,back}
    public boolean last = false;
    public Gamepad gamepad;
    public Button button = null;
    public DetectOnce() {
        //nothing to do here.
    }
    public DetectOnce(Gamepad gamepadIn, Button buttonIn) {
        //nothing to do here.
        gamepad = gamepadIn;
        button = buttonIn;
    }

    public DetectOnce(boolean state) {
        last=state;
    }
    public boolean check(boolean state) {
        if (state==true) {
            if (last==false) {
                last = true;
                return true;
            } else {
                return false;
            }
        }
        last=false;
        return false;
    }

    public boolean pressed() {
        if (button == Button.a) {
            return check(gamepad.a);
        } else if (button == Button.b) {
            return check(gamepad.b);
        } else if (button == Button.x) {
            return check(gamepad.x);
        } else if (button == Button.y) {
            return check(gamepad.y);
        } else if (button == Button.dpad_up) {
            return check(gamepad.dpad_up);
        } else if (button == Button.dpad_down) {
            return check(gamepad.dpad_down);
        } else if (button == Button.dpad_left) {
            return check(gamepad.dpad_left);
        } else if (button == Button.dpad_right) {
            return check(gamepad.dpad_right);
        } else if (button == Button.left_bumper) {
            return check(gamepad.left_bumper);
        } else if (button == Button.right_bumper) {
            return check(gamepad.right_bumper);
        } else if (button == Button.share) {
            return check(gamepad.share);
        } else if (button == Button.back) {
            return check(gamepad.back);
        } else if (button == Button.start) {
            return check(gamepad.start);
        }
        return false;
    }


}
