/* Copyright (c) 2017 FIRST. All rights reserved.
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

import java.util.ArrayList;

/**
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a pushbot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Test: Motor Test Plus", group = "Test")
@Disabled
public class MotorTestPlus extends LinearOpMode {

    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final float NANOSECONDS_PER_SECOND = 1000000000;

    // Define class members
    DcMotor motor;
    double  power   = 0;
    int historySize = 3;
    ArrayList<Integer> positionList = new ArrayList<>();
    ArrayList<Long> timeList = new ArrayList<>();
    ArrayList<Double> speedList = new ArrayList<>();
    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "turret");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the start buttonPOI98
        telemetry.addData(">", "Press Start." );
        telemetry.update();
        waitForStart();
        long time = 0;
        int position = 0;
        double maxSpeed = 0;
        // Ramp motor speeds till stop pressed.
        boolean testing = false;
        boolean accelerating = false;
        boolean decelerating = false;
        long accelerationTicks = 0;
        long decelerationTicks = 0;
        long decelerationStarted = 0;
        while(opModeIsActive()) {
            if (gamepad1.a) {
                if (!testing) {
                    //start test now

                    motor.setPower(1);
                    accelerating = true;
                    decelerating = false;
                    testing = true;
                    positionList.clear();
                    timeList.clear();
                    speedList.clear();
                    accelerationTicks=0;
                    decelerationTicks=0;
                    maxSpeed=0;
                }
            } else {
                if (accelerating) {
                    accelerating=false;
                    decelerating=true;
                    decelerationStarted=positionList.size();
                    motor.setPower(0);
                }
            }


            position = motor.getCurrentPosition();
            time = System.nanoTime();
            positionList.add(position);
            timeList.add(time);
            int last = positionList.size() - 4;
            if (last < 0) {last = 0;}
            double speed = (position - positionList.get(last)) * NANOSECONDS_PER_SECOND / (time - timeList.get(last));
            speedList.add(speed);

            if (accelerating) {
                if (Math.abs(speed) > maxSpeed) {
                    maxSpeed = Math.abs(speed);
                    for (int i=0; i<speedList.size() ; i++) {
                        if (speedList.get(i) > maxSpeed*0.95) {
                            accelerationTicks = i;
                        }
                    }
                }
            } else if (decelerating) {
                if (speed < maxSpeed*0.01) {
                    decelerationTicks = speedList.size() - decelerationStarted;
                    decelerating=false;
                    testing=false;
                }
            }  else {
                power = -gamepad1.left_stick_y;
                motor.setPower(power);

                if (positionList.size() > historySize) {
                    positionList.remove(0);
                }
                if (timeList.size() > historySize) {
                    timeList.remove(0);
                }
                if (Math.abs(speed) > maxSpeed) {
                    maxSpeed = Math.abs(speed);
                }
            }
            // Display the current value
            telemetry.addData("Power", "%5.2f", power);
            telemetry.addData("Position", "%d", motor.getCurrentPosition());
            telemetry.addData("Speed", speed);
            telemetry.addData("MaxSpeed", maxSpeed);
            telemetry.addData("Acceleration Ticks", accelerationTicks);
            telemetry.addData("Deceleration Ticks" , decelerationTicks);
            telemetry.addData("test", testing);
            telemetry.addData("acc", accelerating);
            telemetry.addData("dec", decelerating);

            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the motor to the new power and pause;
            sleep(CYCLE_MS);
            idle();
        }

        // Turn off motor and signal done;
        motor.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
