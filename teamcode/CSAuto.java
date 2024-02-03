/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;


@Autonomous(name = "CSAuto", group = "Autonomous")

public class CSAuto extends LinearOpMode implements MecanumDrive.TickCallback {


    private final MecanumDrive mecanumDrive = new MecanumDrive();


    private final CSRobot robot = new CSRobot();
    private int sleeveCode;

    private enum ScoringDirection {SCORE_LEFT, SCORE_RIGHT}
    private ScoringDirection scoringDirection = ScoringDirection.SCORE_LEFT;
    private int path = 0;
    private CSPropVisionProcessor visionProcessor = new CSPropVisionProcessor();

    public void pixelDrop() {
        robot.setWristBasePosition(0.45);
        sleep(CSConstants.wristMoveMilliseconds);
        robot.moveMotor(robot.elbow, 0, CSConstants.elbowPower, CSConstants.elbowTolerance);
        while ( robot.elbow.isBusy() && opModeIsActive()) {
            mecanumDrive.tickSleep();
        }
        robot.openRightFinger();
        sleep(500);
        mecanumDrive.backward(2,0.5);
        mecanumDrive.forward(5,0.5);
    }

    @Override
    public void runOpMode() {


        robot.init(hardwareMap, telemetry, this);
        mecanumDrive.init(hardwareMap, telemetry, this);
        mecanumDrive.setCountPerDegree(8);
        mecanumDrive.setCountPerInch(40);

        // mecanumDrive.setupTickCallback(this);
//        mecanumDrive.enableDebugWait();

        robot.setDrivetrainMotorDirections(mecanumDrive);
        boolean allianceSelected = false;
        // Tell the driver that initialization is complete.
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(visionProcessor)
                .setCameraResolution(new Size(424, 240))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        ButtonPressDetector pad1pressDetector = new ButtonPressDetector(gamepad1);
        while (opModeInInit()) {
            if (gamepad1.right_bumper) {
                visionProcessor.setAlliance(CSPropVisionProcessor.Alliance.RED);
                allianceSelected=true;
            } else if (gamepad1.left_bumper) {
                visionProcessor.setAlliance(CSPropVisionProcessor.Alliance.BLUE);
                allianceSelected=true;

            }
            if (!allianceSelected) {
                telemetry.addLine("-------------------------------------");
                telemetry.addLine("|        SELECT ALLIANCE NOW!        |");
                telemetry.addLine("-------------------------------------");
                telemetry.addLine("|      NO NOT LATER!  DO IT NOW!     |");
                telemetry.addLine("-------------------------------------");
                telemetry.addLine("Driver controller bumpers set alliance");
                telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
            } else {
                telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
                if (visionProcessor.getAlliance() == CSPropVisionProcessor.Alliance.RED) {
                    telemetry.addLine("<p style=\"text-align:center; background-color:red;\"><big><big><big><b><tt>--RED--</tt></b></big></big></big></p>");
                } else {
                    telemetry.addLine("<p style=\"text-align:center; background-color:blue;\"><big><big><big><b><tt>--BLUE--</tt></b></big></big></big></p>");
                }
                //telemetry.addData("Camera resolution", visionProcessor.getWidth()+ "x"+ visionProcessor.getHeight());
                telemetry.addData("Prop Zone Detected", visionProcessor.getDetectedPropZone());
            }
            telemetry.update();
            sleep(50);
        }

        if (!allianceSelected) {
            visionProcessor.setAlliance(CSPropVisionProcessor.Alliance.RED);
        }

        visionPortal.close();

        //Quit now if stop button was pressed instead of play.
        if (!opModeIsActive()) { return;}

        path=visionProcessor.getDetectedPropZone();

        robot.setArmFree();
        robot.closeLeftFinger();
        robot.closeRightFinger();
        //robot.adjustElbow( CSConstants.elbowReadyForDockUndock);
        robot.moveMotor(robot.elbow, 100, CSConstants.elbowPower, CSConstants.elbowTolerance);


        int turnDirection;

        if (visionProcessor.getAlliance() == CSPropVisionProcessor.Alliance.RED) {
            turnDirection = -1;
            path = 4 - path;
        } else {
            turnDirection = 1;
        }


//        mecanumDrive.leftTurn(5,0.5);
        mecanumDrive.backward(16, 0.5);


        if (path == 1) {
            mecanumDrive.backward(6, 0.2);
            mecanumDrive.turnTo(-90 * turnDirection, 0.5);
            mecanumDrive.forward(24, 0.5);


            robot.setWristBasePosition(0.45);
            sleep(CSConstants.wristMoveMilliseconds);
            robot.moveMotor(robot.elbow, 0, CSConstants.elbowPower, CSConstants.elbowTolerance);
            while ( robot.elbow.isBusy() && opModeIsActive()) {
                mecanumDrive.tickSleep();
            }
            robot.openRightFinger();
            sleep(500);
            mecanumDrive.backward(6,0.5);
            mecanumDrive.forward(5,0.5);

            mecanumDrive.turnTo(0, 0.4);
            mecanumDrive.forward(22, 0.5);
            mecanumDrive.turnTo(-100, 0.4);
            if (turnDirection == -1) {
                mecanumDrive.backward(18, 0.5);
            } else {
                mecanumDrive.forward(18, 0.5);
            }

        } else if (path == 2) {
            mecanumDrive.backward(8, 0.2);
            mecanumDrive.forward(6, 0.2);
            pixelDrop();

            mecanumDrive.turnTo(0, 0.5);
            mecanumDrive.forward(18, 0.5);
            mecanumDrive.turnTo(-100, 0.4);
            if (turnDirection == -1) {
                mecanumDrive.backward(25, 0.5);
            } else {
                mecanumDrive.forward(25, 0.5);
            }

        } else if (path == 3) {
            mecanumDrive.backward(8, 0.2);
            mecanumDrive.turnTo(-90 * turnDirection, 0.4);
            mecanumDrive.forward(4, 0.2);
            pixelDrop();
            mecanumDrive.backward(4, 0.2);

            mecanumDrive.turnTo(0, 0.4);
            mecanumDrive.forward(22, 0.5);
            mecanumDrive.turnTo(-100, 0.4);
            if (turnDirection == -1) {
                mecanumDrive.backward(25, 0.5);
            } else {
                mecanumDrive.forward(25, 0.5);
            }

        }

        robot.startDockingArm();

        while (opModeIsActive()) {
            standardMecanumControls();
            if (pad1pressDetector.wasPressed(ButtonPressDetector.Button.a)) {
                robot.startDockingArm();
            }
            debugOutput();
            robot.doArmStateMachine();
            mecanumDrive.tickSleep();

        }
        mecanumDrive.tickSleep();
    }

    public void debugOutput() {
        telemetry.addData("Prop Zone Detected", visionProcessor.getDetectedPropZone());
        telemetry.addData("Shoulder", robot.getShoulderPosition());
        telemetry.addData("Elbow", robot.getElbowPosition());
        telemetry.addData("Wrist", robot.getWristPosition());
        telemetry.addData("ArmState", robot.getArmState());
        telemetry.addData("ArmPosition" , robot.armPosition);
        mecanumDrive.telemetryMotorPosition();
        telemetry.update();
    }
    public void standardMecanumControls() {
        double speed = (gamepad1.right_trigger * 0.5) + 0.5;
        double fwd = addDeadZone(gamepad1.left_stick_y) * speed;
        double rot = addDeadZone(gamepad1.right_stick_x) * speed;
        double strafe = addDeadZone(gamepad1.left_stick_x);
        strafe = strafe * speed * 1.6;

        if (strafe > 1) {
            strafe = 1;
        } else if (strafe < -1) {
            strafe = -1;
        }
        mecanumDrive.setMotors(strafe, fwd, rot, 1);
    }

    public void tickCallback() {
       // robot.doArmStateMachine();
    }

    double addDeadZone(double input) {
        if (Math.abs(input) < 0.1) {
            return (0.0);
        }
        return (input);
    }

}
