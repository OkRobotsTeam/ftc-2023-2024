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

package org.firstinspires.ftc.old_teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="PoPTurretTest", group ="TeleOp")
@Disabled
public class PoPTTest extends LinearOpMode  {
    private static final int TURRET_COUNT_PER_DEGREE = 135;
    DcMotorEx turret = null;
    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        waitForStart();
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidf = turret.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        pidf.d = 0.01;
        pidf.f = 0.01;

        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,pidf);
        turret.setTargetPositionTolerance(10);

        DetectOnce dpadUpOnce = new DetectOnce(gamepad2, DetectOnce.Button.dpad_up);
        DetectOnce dpadDownOnce = new DetectOnce(gamepad2, DetectOnce.Button.dpad_down);
        DetectOnce dpadLeftOnce = new DetectOnce(gamepad2, DetectOnce.Button.dpad_left);
        DetectOnce dpadRightOnce = new DetectOnce(gamepad2, DetectOnce.Button.dpad_right);
        DetectOnce leftBumperOnce = new DetectOnce(gamepad2, DetectOnce.Button.left_bumper);
        DetectOnce rightBumperOnce = new DetectOnce(gamepad2, DetectOnce.Button.right_bumper);
        DetectOnce startOnce = new DetectOnce(gamepad2, DetectOnce.Button.start);
        DetectOnce backOnce = new DetectOnce(gamepad2, DetectOnce.Button.back);


        while (opModeIsActive()) {

            if (dpadUpOnce.pressed() ) {
                pidf.p = pidf.p * 2;
                updatePIDF(pidf);
            }
            if (dpadDownOnce.pressed()) {
                pidf.p = pidf.p / 2;
                updatePIDF(pidf);
            }
            if (dpadLeftOnce.pressed()) {
                pidf.i = pidf.i * 2;
                updatePIDF(pidf);
            }
            if (dpadRightOnce.pressed()) {
                pidf.i = pidf.i / 2;
                updatePIDF(pidf);
            }
            if (leftBumperOnce.pressed()) {
                pidf.d = pidf.d * 2;
                updatePIDF(pidf);
            }
            if (rightBumperOnce.pressed()) {
                pidf.d = pidf.d / 2;
                updatePIDF(pidf);
            }
            if (startOnce.pressed()) {
                pidf.f = pidf.f * 2;
                updatePIDF(pidf);
            }
            if (backOnce.pressed()) {
                pidf.f = pidf.f / 2;
                updatePIDF(pidf);
            }

            if (gamepad2.b) {
                turret.setTargetPosition(350);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(1);
            }
            if (gamepad2.x) {
                turret.setTargetPosition(0);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(1);
            }
            if (gamepad2.a) {
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turret.setPower(0);
            }
            PIDCoefficients nowPID = turret.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("p", pidf.p);
            telemetry.addData("i", pidf.i);
            telemetry.addData("d", pidf.d);
            //telemetry.addData("f", nowPIDF.f);
            telemetry.addData("Position",turret.getCurrentPosition());
            telemetry.update();
            sleep(50);

        }

    }
    public void updatePIDF(PIDFCoefficients pidf) {
        PIDCoefficients pidNew = new PIDCoefficients(pidf.p, pidf.i, pidf.d);
        //DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)turret.getController();
        //int motorIndex = turret.getPortNumber();
        //motorControllerEx.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_TO_POSITION, pidfNew);
        turret.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidNew);
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
