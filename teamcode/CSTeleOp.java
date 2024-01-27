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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "CS TeleOp", group = "TeleOp")

public class CSTeleOp extends LinearOpMode implements MecanumDrive.TickCallback {

    private int armPosition;
    private final MecanumDrive mecanumDrive = new MecanumDrive();

    private final CSRobot robot = new CSRobot();


    @Override
    public void runOpMode() {
        armPosition=0;
        robot.init(hardwareMap, telemetry, this);
        mecanumDrive.init(hardwareMap, telemetry, this);
        robot.setDrivetrainMotorDirections(mecanumDrive);
        mecanumDrive.setupTickCallback(this);
        ButtonPressDetector pad2pressDetector = new ButtonPressDetector(gamepad2);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        telemetry.addData("Status", "Started");
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Looping");

            standardMecanumControls();
            if (pad2pressDetector.wasPressed(ButtonPressDetector.Button.a)) {
                robot.startDockingArm();

            } else if (pad2pressDetector.wasPressed(ButtonPressDetector.Button.b)) {
                robot.startUndockingArm();
            }

            if (pad2pressDetector.wasPressed(ButtonPressDetector.Button.dpad_down) ) {
                robot.armDown();
            }
            if (pad2pressDetector.wasPressed(ButtonPressDetector.Button.dpad_up) ) {
                robot.armUp();
            }

            // Handle opening and closing the fingers
            if (gamepad2.left_bumper) {
                robot.openLeftFinger();
            } else {
                robot.closeLeftFinger();
            }
            if (gamepad2.right_bumper) {
                robot.openRightFinger();
            } else {
                robot.closeRightFinger();
            }

            //Intake Controls
            if (gamepad2.right_trigger > 0.3) {
                robot.runIntakeReverse();
            } else {
                robot.runIntakeForward(gamepad2.left_trigger);
            }

            if (pad2pressDetector.wasPressed(ButtonPressDetector.Button.x)) {
                robot.deployEndgame();
            }
            // Process endgame firing
            robot.doArmStateMachine();
            robot.endgameTick();

            telemetry.addData("Shoulder", robot.getShoulderPosition());
            telemetry.addData("Elbow", robot.getElbowPosition());
            telemetry.addData("Wrist", robot.getWristPosition());
            telemetry.addData("ArmState", robot.getArmState());
            telemetry.update();

        }
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


    double addDeadZone(double input) {
        if (Math.abs(input) < 0.1) {
            return (0.0);
        }
        return (input);
    }

    public void tickCallback() {

    }

}
