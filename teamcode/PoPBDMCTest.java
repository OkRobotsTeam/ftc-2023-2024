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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import static org.firstinspires.ftc.teamcode.ButtonPressDetector.Button;

@TeleOp(name="PoPTurretTestBDMC", group ="TeleOp")
@Disabled
public class PoPBDMCTest extends LinearOpMode  {
    private static final int TURRET_COUNT_PER_DEGREE = 135;
    BrakingDistanceMotorController turret = null;

    @Override
    public void runOpMode() {
        //builtin encoder
        //turret = new BrakingDistanceMotorControler(hardwareMap,"turret", 560);
        //through bore encoder
        turret = new BrakingDistanceMotorController(hardwareMap,"turret", 560);
        waitForStart();
        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setLabel("turret");
        long brakingSpeed = 4000;
        turret.setBrakingSpeed(brakingSpeed);

        turret.setAccelerationStartPower(0.3);
        ButtonPressDetector pressDetector = new ButtonPressDetector(gamepad2);

        int decelerationTicks = 10;
        turret.setDecelerationTicks(decelerationTicks);


        while (opModeIsActive()) {
            if (pressDetector.wasToggled(Button.dpad_up)) {
                System.out.println("Dpad Up was toggled");
            }
            if (pressDetector.wasToggled(Button.left_stick_up)) {
                System.out.println("left_stick_up Up was toggled");
            }
            if (pressDetector.wasToggled(Button.right_stick_button)) {
                System.out.println("right_stick_button was toggled");
            }


            if (pressDetector.wasPressed(Button.dpad_up) ) {
                decelerationTicks++;
                turret.setDecelerationTicks(decelerationTicks);
            }
            if (pressDetector.wasPressed(Button.dpad_down)) {
                decelerationTicks--;
                turret.setDecelerationTicks(decelerationTicks);

            }
            if (pressDetector.wasPressed(Button.left_trigger)) {
                brakingSpeed=brakingSpeed + 100;
                turret.setBrakingSpeed(brakingSpeed);

            }
            if (pressDetector.wasPressed(Button.right_trigger)) {
                brakingSpeed=brakingSpeed - 100;
                turret.setBrakingSpeed(brakingSpeed);
            }
            if (pressDetector.wasPressed(Button.dpad_left)) {
                turret.setAccelerationTicks(turret.getAccelerationTicks()+1);
            }
            if (pressDetector.wasPressed(Button.dpad_right)) {
                turret.setAccelerationTicks(turret.getAccelerationTicks()-1);
            }

            if (pressDetector.wasPressed(Button.left_bumper)) {
                turret.setAccelerationStartPower(turret.getAccelerationStartPower()*0.8);
            }
            if (pressDetector.wasPressed(Button.right_bumper)) {
                turret.setAccelerationStartPower(turret.getAccelerationStartPower()*1.25);
            }

            if (pressDetector.wasPressed(Button.b)) {
                turret.runToPosition(175, 1);
            }
            if (pressDetector.wasPressed(Button.x)) {
                turret.runToPosition(-175, 1);
            }
            if (pressDetector.wasPressed(Button.a)) {
                turret.stop();
            }
            turret.onTick();
            telemetry.addData("Deceleration Ticks",turret.getDecelerationTicks());
            telemetry.addData("Acceleration Ticks",turret.getAccelerationTicks());
            telemetry.addData("Acceleration Start Power",turret.getAccelerationStartPower());
            telemetry.addData("Braking Speed",turret.getBrakingSpeed());

            telemetry.addData("Position",turret.getCurrentPosition());
            telemetry.update();
            sleep(50);

        }

    }
    public void turnTurretTo(double degrees){
        if (degrees > 180) {
            degrees = 180;
        } else if (degrees < -180) {
            degrees = -180;
        }
        int destination = (int) (TURRET_COUNT_PER_DEGREE * degrees);
    }

    private boolean isTurretAtDestination() {
        return (Math.abs(getTurretLeft()) < 150);
    }
    private int getTurretLeft() {
        return (turret.getTargetPosition() - turret.getCurrentPosition());
    }

}
