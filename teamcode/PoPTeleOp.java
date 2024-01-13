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

import static org.firstinspires.ftc.teamcode.ButtonPressDetector.Button.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "PoPTeleOp", group = "TeleOp")

public class PoPTeleOp extends LinearOpMode implements MecanumDrive.TickCallback {


    private final MecanumDrive mecanumDrive = new MecanumDrive();

    private final PoPRobot robot = new PoPRobot();
    private boolean turretIsMoving = false;
    private boolean elevatorIsMoving = false;

    private enum States {S0, S1, PICKUP, S2POINT5, SCORETURN, S4, S5}

    ;
    private States autoScoreState = States.S1;
    private long armReleaseTimer = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, this);
        mecanumDrive.init(hardwareMap, telemetry, this);
        robot.setMotorDirections(mecanumDrive);
        mecanumDrive.setupTickCallback(this);
        ButtonPressDetector pad2pressDetector = new ButtonPressDetector(gamepad2);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        while (!isStarted()) {
            robot.getSleevePosition();
            sleep(50);
        }
        waitForStart();
        robot.stopVision();
        //START OF FFAuto
        // Auto Position 2 Fancy


        //robot.setDoorPosition(FFRobot.doorPosition.PICKUP);
        while (opModeIsActive()) {

            //MANIPULATOR
            if (gamepad2.right_bumper && false) {
                switch (autoScoreState) {
                    case S0:
                        turnTurretTo(90);
                        robot.turnArmTo(120);
                        robot.setWrist(0.5, 0);
                        autoScoreState = States.S1;
                        turretIsMoving = true;
                        break;
                    case S1:
                        if (!robot.turretTickResult()) {
                            robot.turnArmTo(250);
                            robot.setWrist(0.8, 0);
                            robot.clawRelease();
                            autoScoreState = States.PICKUP;
                            turretIsMoving = false;
                        }
                        break;
                    case PICKUP:
                        if (robot.isArmAtDestination()) {
                            robot.clawGrab();
                            armReleaseTimer = System.currentTimeMillis();
                            autoScoreState = States.S2POINT5;
                        }
                        break;
                    case S2POINT5:
                        if ((System.currentTimeMillis() - armReleaseTimer) > 1000) {
                            robot.turnArmTo(120);
                            robot.setWrist(0.5, 0);
                            autoScoreState = States.SCORETURN;
                        }
                        break;
                    case SCORETURN:
                        if (robot.getArmDegree() < 180) {
                            turnTurretTo(-90);
                            robot.setWrist(0.3, 0);
                            autoScoreState = States.S4;
                            turretIsMoving = true;
                        }
                        break;
                    case S4:
                        if (!robot.turretTickResult()) {
                            robot.clawRelease();
                            armReleaseTimer = System.currentTimeMillis();
                            turretIsMoving = false;
                            autoScoreState = States.S5;
                        }
                        break;
                    case S5:
                        if ((System.currentTimeMillis() - armReleaseTimer) > 1000) {
                            autoScoreState = States.S0;
                        }
                        break;
                }
            } else {
                if (pad2pressDetector.wasPressed(dpad_up)) {
                    turnTurretTo(0);
                } else if (pad2pressDetector.wasPressed(dpad_down)) {
                    turnTurretTo(180);
                } else if (pad2pressDetector.wasPressed(dpad_left)) {
                    turnTurretTo(-90);
                } else if (pad2pressDetector.wasPressed(dpad_right)) {
                    turnTurretTo(90);
                } else if (pad2pressDetector.wasPressed(dpad_up_left)) {
                    turnTurretTo(-45);
                } else if (pad2pressDetector.wasPressed(dpad_up_right)) {
                    turnTurretTo(45);
                } else if (pad2pressDetector.wasPressed(dpad_down_left)) {
                    turnTurretTo(-135);
                } else if (pad2pressDetector.wasPressed(dpad_down_right)) {
                    turnTurretTo(135);
                } else if (turretIsMoving) {
                    if (!robot.turretTickResult() || gamepad2.back) {
                        turretIsMoving = false;
                        //robot.turretFreeMoveMode();
                    }
                } else {
                    robot.setTurretPower(-gamepad2.left_stick_x / 1.5);
                }
                if (gamepad2.left_bumper) {
                    robot.waveTick();
                    System.out.println("Main wavetick");
                } else if (gamepad2.back) {
                    robot.waveTick();
                    System.out.println("Alternate wavetick");
                }


                if (gamepad2.left_trigger > 0.3) {
                    if (pad2pressDetector.wasPressed(a)) {
                        robot.turnArmTo(robot.getArmTargetDegree() + 10);
                        robot.setWristBasePosition(robot.getWristBasePosition() + 0.03);
                    } else if (gamepad2.x) {
                        robot.setWristBasePosition(robot.getWristBasePosition() + 0.005);
                    } else if (gamepad2.b) {
                        robot.setWristBasePosition(robot.getWristBasePosition() - 0.005);
                    } else if (pad2pressDetector.wasPressed(y)) {
                        robot.turnArmTo(robot.getArmTargetDegree() - 10);
                        robot.setWristBasePosition(robot.getWristBasePosition() - 0.03);
                    }

                } else {
                    if (gamepad2.a) {
                        robot.turnArmTo(250);
                        robot.setWristBasePosition(0.73);
                    } else if (gamepad2.x) {
                        robot.turnArmTo(220);
                        robot.setWristBasePosition(0.66);
                    } else if (gamepad2.b) {
                        robot.turnArmTo(145);
                        robot.setWristBasePosition(0.38);
                    } else if (gamepad2.y) {
                        robot.turnArmTo(90);
                        robot.setWristBasePosition(0.2);
                    } else if (gamepad2.start) {
                        robot.turnArmTo(5);
                        robot.setWristBasePosition(0.2);
                    }
                }

                if (gamepad2.right_bumper) {
                    robot.clawRelease();
                } else {
                    robot.clawGrab();
                }
                robot.setWristOffset(gamepad2.right_trigger / 3);
            }
            if (elevatorIsMoving) {
                if (gamepad2.back) {
                    robot.stopElevator();
                    elevatorIsMoving = false;
                } else {
                    if (robot.elevatorTickResult() == false) {
                        //done moving
                        elevatorIsMoving = false;
                    }
                    telemetry.addData("elevator power", "Auto");
                }
            } else {
                //robot.setElevatorPowerWithLimitSwitches(-gamepad2.right_stick_y);
                //telemetry.addData("elevator power", -gamepad2.right_stick_y);
            }

            standardMecanumControls();
            telemetry.addData("wristPosition", robot.getWristPosition());
            telemetry.addData("turretIsMoving", turretIsMoving);
            telemetry.addData("turretPosition", robot.getTurretPosition());
            telemetry.addData("turretPower", robot.turret.getPower());
            telemetry.addData("Turret Position", robot.getTurretPosition());
            telemetry.addData("Arm Position", robot.getArmPosition());
            telemetry.addData("Elevator Position", robot.getElevatorPosition());
            telemetry.update();
        }
    }


    public void standardMecanumControls() {
        double speed = 1;
        //speed = 1 - (gamepad1.right_trigger * 0.5) + 0.5;
        speed = (gamepad1.right_trigger * 0.5) + 0.5;
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

    public void turnTurretTo(double degrees) {
        robot.turnTurretTo(-degrees, 1);
        turretIsMoving = true;
    }

    public void tickCallback() {
    }

    double addDeadZone(double input) {
        if (Math.abs(input) < 0.1) {
            return (0.0);
        }
        return (input);
    }

}
