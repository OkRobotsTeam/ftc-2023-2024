package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CSRobot {


    private DcMotorEx shoulder = null;
    private DcMotorEx elbow = null;

    private Servo wrist = null;
    private Servo leftFinger = null;
    private Servo rightFinger = null;
    private CRServo launcher = null;
    long wristStartedMoving = 0;
    private long endgameDeployStartTime = -1;
    private Telemetry telemetry = null;

    double wristPosition = 0;

    private enum armStates {DOCKED, DOCKING2, DOCKING1, FREE, UNDOCKING}

    private armStates armState = armStates.DOCKED;

    /*
    Transition from FREE to DOCKED
    (Moving the arm back into the robot)
    FREE: Can move around freely, outside of the robot
    DOCKING1: The wrist is in the upright position to allow it to get into the robot body
    DOCKING2: The wrist is in the upright position and the arm is hovering just above the inside of the robot
    DOCKED: The wrist is in the horizontal position and the arm is fully down into the robot
    */

    /*
    Transition from DOCKED to FREE
    (Moving the arm out of the robot)
    DOCKED: The wrist is in the horizontal position and the arm is fully down into the robot
    */


    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {
        telemetry = telemetryIn;
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        elbow = hardwareMap.get(DcMotorEx.class, "elbow");

        wrist = hardwareMap.get(Servo.class, "wrist");
        leftFinger = hardwareMap.get(Servo.class, "left_finger");
        rightFinger = hardwareMap.get(Servo.class, "right_finger");
        launcher = hardwareMap.get(CRServo.class, "launcher");

        shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void openLeftFinger() {
        leftFinger.setPosition(CSConstants.leftFingerOpenPosition);
    }

    public void closeLeftFinger() {
        leftFinger.setPosition(CSConstants.leftFingerClosedPosition);
    }

    public void openRightFinger() {
        rightFinger.setPosition(CSConstants.rightFingerOpenPosition);
    }

    public void closeRightFinger() {
        rightFinger.setPosition(CSConstants.rightFingerClosedPosition);
    }

    public armStates getArmState() {
        return armState;
    }

    public void deployEndgame() {
        endgameDeployStartTime = System.currentTimeMillis();
    }

    public void endgameTick() {
        if (endgameDeployStartTime >= 0) {
            launcher.setPower(1);
        } else if (((System.currentTimeMillis() - endgameDeployStartTime) / 1000.0d) > CSConstants.endgameRuntimeSeconds) {
            launcher.setPower(0);
        }
    }

    public boolean isAtDestination(DcMotorEx motor) {
        return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) <= motor.getTargetPositionTolerance();
    }

    public void moveMotor(DcMotorEx motor, int target, double power, int tolerance) {
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        motor.setTargetPositionTolerance(tolerance);
    }

    public void startDockingArm() {
        if (armState != armStates.FREE) {
            //only works in FREE state
            return;
        }

        //start moving shoulder and elbow up, transition to undocking state
        moveMotor(shoulder, CSConstants.shoulderReadyForDockUndock, CSConstants.shoulderPower, CSConstants.shoulderTolerance);
        moveMotor(elbow, CSConstants.elbowReadyForDockUndock, CSConstants.elbowPower, CSConstants.elbowTolerance);
        wrist.setPosition(CSConstants.wristUpright);
        armState = armStates.DOCKING1;
    }

    public void startUndockingArm() {
        if (armState != armStates.DOCKED) {
            //only works in DOCKED state
            return;
        }

        //start moving shoulder and elbow down, transition to docking1 state
        moveMotor(shoulder, CSConstants.shoulderReadyForDockUndock, CSConstants.shoulderPower, CSConstants.shoulderTolerance);
        moveMotor(elbow, CSConstants.elbowReadyForDockUndock, CSConstants.elbowPower, CSConstants.elbowTolerance);
        armState = armStates.UNDOCKING;
    }


    public void armStateMachine() {
        switch (armState) {
            case DOCKED:
                //allow the user to load pixels
                break;
            case UNDOCKING:
                if (isAtDestination(elbow) && isAtDestination(shoulder)) {
                    armState = armStates.FREE;
                    moveMotor(shoulder, CSConstants.shoulderDefaultFreePosition, CSConstants.shoulderPower, CSConstants.shoulderTolerance);
                    moveMotor(elbow, CSConstants.elbowDefaultFreePosition, CSConstants.elbowPower, CSConstants.elbowTolerance);
                    wrist.setPosition(CSConstants.wristUpright);
                }

                //if shoulder and elbow are up far enough, transition to free
                break;
            case FREE:
                //allow user to set arm position
                break;
            case DOCKING1:
                if (isAtDestination(elbow) && isAtDestination(shoulder)) {
                    armState = armStates.DOCKING2;
                    wrist.setPosition(CSConstants.wristDocking);
                    wristStartedMoving = System.currentTimeMillis();
                }
                //if shoulder and elbow are down far enough, start moving wrist to docking position, transition to ready to dock
                break;
            case DOCKING2:
                if (System.currentTimeMillis() - wristStartedMoving > CSConstants.wristMoveMilliseconds) {
                    moveMotor(shoulder, CSConstants.shoulderDocked, CSConstants.shoulderPower, CSConstants.shoulderTolerance);
                    moveMotor(elbow, CSConstants.elbowDocked, CSConstants.elbowPower, CSConstants.elbowTolerance);                }
                //if enough time has passed for wrist to move, start dropping shoulder and elbow all the way down, transition to docked
                break;
        }
    }

    public void elbowUp() {
        elbow.setTargetPosition(elbow.getTargetPosition() + CSConstants.elbowAdjustmentSize);
    }

    public void elbowDown() {
        elbow.setTargetPosition(elbow.getTargetPosition() - CSConstants.elbowAdjustmentSize);
    }

    public void moveArm(int shoulderPositionIn, int elbowPositionIn, double wristPositionIn) {
        if (armState != armStates.FREE) {
            //only works in FREE state
            return;
        }
        shoulder.setTargetPosition(shoulderPositionIn);
        elbow.setTargetPosition(elbowPositionIn);
        wrist.setPosition(wristPositionIn);
        wristPosition = wristPositionIn;
    }

    public double getShoulderPosition() {
        return shoulder.getCurrentPosition();
    }

    public double getElbowPosition() {
        return elbow.getCurrentPosition();
    }

    public double getWristPosition() {
        return wristPosition;
    }

    public void setDrivetrainMotorDirections(MecanumDrive mecanumDrive) {
        mecanumDrive.setMotorDirections(FORWARD, REVERSE, FORWARD, REVERSE);
    }
}