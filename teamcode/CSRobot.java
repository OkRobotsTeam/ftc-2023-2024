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


    public DcMotorEx shoulder = null;
    public DcMotorEx elbow = null;
    private DcMotorEx flippers = null;
    private DcMotorEx belts = null;

    private Servo wrist = null;
    private Servo leftFinger = null;
    private Servo rightFinger = null;
    private CRServo launcher = null;
    long wristStartedMoving = 0;
    private long endgameDeployStartTime = -1;
    private Telemetry telemetry = null;

    double wristBasePosition = 0;
    public int armPosition = 0;
    int shoulderTarget = 0;
    int elbowTarget = 0;
    int shoulderAdjust = 0;
    int elbowAdjust = 0;
    double wristAdjust = 0;


    enum armStates {DOCKED, DOCKING2, DOCKING1, FREE, UNDOCKING, DOCKING3, UNDOCKING2}

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

        flippers = hardwareMap.get(DcMotorEx.class, "intake");
        belts = hardwareMap.get(DcMotorEx.class, "belts");

        wrist = hardwareMap.get(Servo.class, "wrist");
        leftFinger = hardwareMap.get(Servo.class, "left_finger");
        rightFinger = hardwareMap.get(Servo.class, "right_finger");
        launcher = hardwareMap.get(CRServo.class, "launcher");

        shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shoulder.setDirection(REVERSE);
        elbow.setDirection(REVERSE);
        wrist.setPosition(CSConstants.wristPickup);
    }


    public void openLeftFinger() {
        leftFinger.setPosition(CSConstants.leftFingerOpenPosition);
    }

    public void closeLeftFinger() {
        if (armState == armStates.DOCKED) {
            return;  // Can't close fingers while docked
        }
        leftFinger.setPosition(CSConstants.leftFingerClosedPosition);
    }

    public void openRightFinger() {
        rightFinger.setPosition(CSConstants.rightFingerOpenPosition);
    }

    public void closeRightFinger() {
        if (armState == armStates.DOCKED) {
            return;  // Can't close fingers while docked
        }
        rightFinger.setPosition(CSConstants.rightFingerClosedPosition);
    }
    public void elbowAdjustUp() {
        elbowAdjust-=20;
        moveArmTargetWithAdjustments();
    }
    public void elbowAdjustDown() {
        elbowAdjust+=20;
        moveArmTargetWithAdjustments();
    }
    public void shoulderAdjustOut() {
        shoulderAdjust-=20;
        moveArmTargetWithAdjustments();
    }
    public void shoulderAdjustBack() {
        shoulderAdjust +=20;
        moveArmTargetWithAdjustments();
    }
    public void wristAdjustUp(){
        wristAdjust -=0.01;
        setWristPositionWithAdjust();
    }

    public void wristAdjustDown() {
        wristAdjust +=0.01;
        setWristPositionWithAdjust();
    }


    public void runIntakeForward(){
        flippers.setPower(CSConstants.flipperPower);
        belts.setPower(CSConstants.beltPower);
    }

    public void runIntakeForward(double speed){
        flippers.setPower(speed);
        belts.setPower(speed);
    }


    public void runIntakeReverse(){
        flippers.setPower(-CSConstants.flipperPower);
        belts.setPower(-CSConstants.beltPower);
    }

    public void stopIntake(){
        flippers.setPower(0);
        belts.setPower(0);
    }

    public armStates getArmState() {
        return armState;
    }

    public void deployEndgame() {
        endgameDeployStartTime = System.currentTimeMillis();
    }

    public void endgameTick() {

        if (((System.currentTimeMillis() - endgameDeployStartTime) / 1000.0d) > CSConstants.endgameRuntimeSeconds) {
            launcher.setPower(0);
        } else if (endgameDeployStartTime >= 0) {
        launcher.setPower(1);
        }
    }

    public void armUp() {
       if (armPosition == 0) {
           startUndockingArm();
       } else {
           armPosition++;
       }

       if (armPosition > CSConstants.numArmPositions) {
            armPosition = CSConstants.numArmPositions;
       }
       resetAdjusts();
       moveArmTarget();
    }
    public void armDown() {
        if (armPosition == 0) {
            //Do nothing.  We are already docking.
        } else if (armPosition == 1) {
            startDockingArm();
            armPosition = 0;
        } else {
            armPosition--;
            if (armPosition < 0) {
                armPosition = 0;
            }
            moveArmTarget();
        }

    }

    public void moveArmTarget() {
        //setShoulder and elbow target positions
        resetAdjusts();
        moveArmTargetWithAdjustments();
    }

    public void moveArmTargetWithAdjustments() {
        shoulderTarget = CSConstants.armPositions[0][armPosition];
        elbowTarget = CSConstants.armPositions[1][armPosition];
        if (armState == armStates.FREE) {
            elbow.setTargetPosition(elbowTarget+elbowAdjust);
            shoulder.setTargetPosition(shoulderTarget+shoulderAdjust);
            wristBasePosition = CSConstants.wristPositions[armPosition];
            wrist.setPosition(wristBasePosition + wristAdjust);
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
    public boolean servoDone() {
        return (System.currentTimeMillis() - wristStartedMoving > CSConstants.wristMoveMilliseconds);
    }
    public void startDockingArm() {
        if (armState != armStates.FREE) {
            //only works in FREE state
            return;
        }
        armPosition=0;
        shoulderAdjust=0;
        //start moving shoulder and elbow up, transition to undocking state
        moveMotor(shoulder, CSConstants.shoulderReadyForDockUndock, CSConstants.shoulderPower, CSConstants.shoulderTolerance);
        moveMotor(elbow, CSConstants.elbowReadyForDockUndock, CSConstants.elbowPower, CSConstants.elbowTolerance);
        wrist.setPosition(CSConstants.wristDocking);
        wristStartedMoving = System.currentTimeMillis();
        armState = armStates.DOCKING1;
    }

    public void resetAdjusts() {
        elbowAdjust=0;
        shoulderAdjust=0;
        wristAdjust=0;
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
        armPosition = 1;
    }


    public void doArmStateMachine() {
        switch (armState) {
            case DOCKED:
                //allow the user to load pixels
                break;
            case UNDOCKING:

                if (isAtDestination(elbow) && isAtDestination(shoulder)) {
                    telemetry.addData("Undocking", "Undocking2");
                    armState = armStates.UNDOCKING2;
                    wrist.setPosition(CSConstants.wristDocking);
                    wristStartedMoving = System.currentTimeMillis();

                } else {
                    telemetry.addData("Undocking", "pending");
                }
                break;
            case UNDOCKING2:
                telemetry.addData("Undocking", "Undocking2");
                if (System.currentTimeMillis() - wristStartedMoving > CSConstants.wristMoveMilliseconds) {
                    moveMotor(shoulder, CSConstants.shoulderDefaultFreePosition, CSConstants.shoulderPower, CSConstants.shoulderTolerance);
                    moveMotor(elbow, CSConstants.elbowDefaultFreePosition, CSConstants.elbowPower, CSConstants.elbowTolerance);
                    armState = armStates.FREE;
                    moveArmTarget();
                }

                //if shoulder and elbow are up far enough, transition to free
                break;
            case FREE:
                //allow user to set arm position
                break;
            case DOCKING1:
                if (isAtDestination(elbow) && isAtDestination(shoulder) && servoDone()) {
                    //DOCKING2 Done, Transition to DOCKING3
                    wrist.setPosition(CSConstants.wristPickup);
                    armState=armStates.DOCKING2;
                    wristStartedMoving=System.currentTimeMillis();

                }
                //if shoulder and elbow are down far enough, start moving wrist to docking position, transition to ready to dock
                break;


            case DOCKING2:
                if (servoDone()) {
                    //DOCKING3 Done, Transition to DOCKED
                    moveMotor(shoulder, CSConstants.shoulderDocked, CSConstants.shoulderPower, CSConstants.shoulderTolerance);
                    moveMotor(elbow, CSConstants.elbowDocked, 0.1, CSConstants.elbowTolerance);
                    //if enough time has passed for wrist to move, start dropping shoulder and elbow all the way down, transition to docked
                    armState=armStates.DOCKED;
                    break;
                }
        }
    }

    public void moveArm(int shoulderPositionIn, int elbowPositionIn, double wristPositionIn) {
        if (armState != armStates.FREE) {
            //only works in FREE state
            return;
        }
        shoulder.setTargetPosition(shoulderPositionIn);
        elbow.setTargetPosition(elbowPositionIn);
        wrist.setPosition(wristPositionIn);
        wristBasePosition = wristPositionIn;
    }

    public void setWristBasePosition(double positionIn) {
        wristBasePosition = positionIn;
        wristAdjust = 0;
        wrist.setPosition(wristBasePosition);
    }

    public void setWristPositionWithAdjust() {
        if (armState != armStates.FREE) {return;}
        wrist.setPosition(wristBasePosition + wristAdjust);
    }
    public double getShoulderPosition() {
        return shoulder.getCurrentPosition();
    }

    public double getElbowPosition() {
        return elbow.getCurrentPosition();
    }

    public double getWristPosition() {
        return wristBasePosition+wristAdjust;
    }

    public void setDrivetrainMotorDirections(MecanumDrive mecanumDrive) {
        mecanumDrive.setMotorDirections(FORWARD, REVERSE, FORWARD, REVERSE);
    }
}