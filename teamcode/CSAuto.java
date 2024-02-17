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


@Autonomous(name = "CSAuto", group = "Autonomous")

public class CSAuto extends LinearOpMode implements MecanumDrive.TickCallback {


    private final MecanumDrive mecanumDrive = new MecanumDrive();

    private enum Path {NONE, PARK, SCORE, TEST};


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
        while (robot.elbow.isBusy() && opModeIsActive()) {
            mecanumDrive.tickSleep();
        }
        robot.openRightFinger();
        sleep(500);
    }

    @Override
    public void runOpMode() {


        robot.init(hardwareMap, telemetry, this);
        mecanumDrive.init(hardwareMap, telemetry, this);
        mecanumDrive.setCountPerDegree(9.16);
        mecanumDrive.setCountPerInch(40);

        // mecanumDrive.setupTickCallback(this);
//        mecanumDrive.enableDebugWait();

        robot.setDrivetrainMotorDirections(mecanumDrive);
        boolean allianceSelected = false;
        boolean pathSelected = false;
        boolean backstageSelected = false;
        boolean backstage = true;
        Path afterPath = Path.SCORE;

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
                allianceSelected = true;
            } else if (gamepad1.left_bumper) {
                visionProcessor.setAlliance(CSPropVisionProcessor.Alliance.BLUE);
                allianceSelected = true;

            }
            if (!allianceSelected) {

                telemetry.addLine("<p style=\"text-align:center;\"><big><big><b><tt>SELECT ALLIANCE NOW!</tt></b></big></big></p>");
                //telemetry.addLine("<tt>Driver controller bumpers set alliance</tt>");


            } else {
                telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
                if (visionProcessor.getAlliance() == CSPropVisionProcessor.Alliance.RED) {
                    telemetry.addLine("<p style=\"text-align:center; background-color:red;\"><big><big><b><tt>--RED--</tt></b></big></big></p>");
                } else {
                    telemetry.addLine("<p style=\"text-align:center; background-color:blue;\"><big><big><b><tt>--BLUE--</tt></b></big></big></p>");
                }
            }
            if (gamepad1.x) {
                afterPath = Path.PARK;
            } else if (gamepad1.y) {
                afterPath = Path.SCORE;
            } else if (gamepad1.b) {
                afterPath = Path.NONE;
            } else if (gamepad1.a) {
                //afterPath = Path.TEST;
            }
            if (gamepad1.dpad_left) {
                backstage = true;
            } else if (gamepad1.dpad_right) {
                backstage = false;
            }
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
            telemetry.addLine("<p style=\"text-align:center;\"><big><big><b><tt> Backstage: " + backstage + "</tt></b></big></big></p>");
            telemetry.addLine("<p style=\"text-align:center;\"><big><big><b><tt> Path: " + afterPath + "</tt></b></big></big></p>");
            telemetry.addLine("<p style=\"text-align:center;\"><big><big><b><tt>Prop Zone Detected: " + visionProcessor.getDetectedPropZone() + " </tt></b></big></big></p>");

            telemetry.update();
            sleep(50);
        }



        //Quit now if stop button was pressed instead of play.
        if (!opModeIsActive()) {
            return;
        }
        //**PLAY BUTTON HIT**
        if (gamepad1.a) {
            mecanumDrive.enableDebugWait();
        }
        robot.setArmFree();
        robot.closeLeftFinger();
        robot.closeRightFinger();
        robot.moveElbow( CSConstants.elbowReadyForDockUndock,CSConstants.elbowPower);


        int turnDirection;
        boolean isRed;
        if (visionProcessor != null) {
            path = visionProcessor.getDetectedPropZone();
            isRed = visionProcessor.getAlliance() == CSPropVisionProcessor.Alliance.RED;
        } else {
            isRed = false;
            path=2;
        }
        visionPortal.close();


        if (isRed ^ !backstage) {
            turnDirection = -1;
            path = 4 - path;
        } else {
            turnDirection = 1;
        }

        if (afterPath == Path.TEST) {
            //mecanumDrive.leftTurn(360,0.5);
            mecanumDrive.backward(20,0.5);
            mecanumDrive.turnTo(180, 0.5);
            mecanumDrive.backward(20, 0.5);
            mecanumDrive.turnTo(0,0.5);
            return;
        }

//        mecanumDrive.leftTurn(5,0.5);
        mecanumDrive.backward(16, 0.5);

        double frontTowardsTruss = 90 * turnDirection;
        double frontAwayFromTruss = -90 * turnDirection;

        if (path == 1) {
            mecanumDrive.backward(11, 0.2);
            mecanumDrive.turnTo(frontAwayFromTruss, 0.5);
            mecanumDrive.forward(24, 0.5);

            pixelDrop();

            mecanumDrive.backward(6, 0.5);
            mecanumDrive.forward(7, 0.5);
            mecanumDrive.turnTo(frontAwayFromTruss, 0.5);

        } else if (path == 2) {
            mecanumDrive.backward(6, 0.2);
            pixelDrop();
            mecanumDrive.backward(2, 0.5);
            mecanumDrive.forward(3, 0.5);

            mecanumDrive.turnTo(frontTowardsTruss, 0.4);
            mecanumDrive.backward(25, 0.5);

        } else if (path == 3) {
            mecanumDrive.backward(11, 0.2);
            mecanumDrive.turnTo(frontAwayFromTruss, 0.4);
            mecanumDrive.forward(4, 0.2);
            pixelDrop();
            mecanumDrive.backward(8, 0.5);
            mecanumDrive.forward(9, 0.5);
            mecanumDrive.turnTo(frontTowardsTruss, 0.4);
            mecanumDrive.backward(21, 0.5);
        }
        if (afterPath == Path.SCORE) {
            if (backstage) {
                robot.moveElbow(600, 0.5);
                double angle = frontTowardsTruss - (((path-1.5) * 30) * turnDirection);
                mecanumDrive.turnTo(angle, 0.7);

                    robot.moveShoulder(1130, 0.5);
                    robot.setWrist(0.34);
                    mecanumDrive.backward(9, 0.5);

                    mecanumDrive.turnTo(frontTowardsTruss, 0.5);
                    mecanumDrive.turnTo(frontTowardsTruss, 0.5);

                    mecanumDrive.backward(4, 0.4);
                    robot.openLeftFinger();
                    sleep(500);
                    mecanumDrive.forward(3, 0.5);
                    robot.moveArm(1);

                    mecanumDrive.turnTo(180, 0.5);
                    if (path == 1) {
                        mecanumDrive.backward(20, 0.5);
                    } else if (path == 2) {
                        mecanumDrive.backward(24, 0.5);
                    } else if (path == 3) {
                        mecanumDrive.backward(28, 0.5);
                    }
                    strafeMiddle(isRed, 24);
            } else {
                //farstage
                mecanumDrive.turnTo(0, 0.5);


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

    public void strafeWall(boolean isRed, double distance ) {
        if (isRed) {
            mecanumDrive.strafeLeft(distance, 1);
        } else {
            mecanumDrive.strafeRight(distance, 1);
        }
    }
    public void strafeMiddle(boolean isRed, double distance) {
        strafeWall(!isRed, distance);
    }
    public void debugOutput() {
        telemetry.addData("Prop Zone Detected", visionProcessor.getDetectedPropZone());
        telemetry.addData("Shoulder", robot.getShoulderPosition());
        telemetry.addData("Elbow", robot.getElbowPosition());
        telemetry.addData("Wrist", robot.getWristPosition());
        telemetry.addData("ArmState", robot.getArmState());
        telemetry.addData("ArmPosition", robot.armPosition);
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
