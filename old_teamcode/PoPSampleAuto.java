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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="PoP Sample Auto", group ="Autonomous")
@Disabled


public class PoPSampleAuto extends LinearOpMode {


    private MecanumDrive mecanumDrive = new MecanumDrive();
    //private UGRobot robot = new UGRobot();
    //This is Object Detection (OD)
    //private UGObjectDetector OD = new UGObjectDetector();
    //private int DWAS = 2;//Duck Wheel Acceleration Speed




    @Override public void runOpMode()
    {

        mecanumDrive.init(hardwareMap, telemetry, this);
        mecanumDrive.setMotorDirections(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        mecanumDrive.setCountPerDegree(9);
        //robot.init(hardwareMap, telemetry, this);
        //This is Object Detection (OD)
        //OD.init(hardwareMap, telemetry,this);
        //mecanumDrive.setupTickCallback(robot);
        //robot.multishotDelay = 225;


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        waitForStart();
        mecanumDrive.enableDebugWait();

        mecanumDrive.forward(14,0.6);
       mecanumDrive.forward(38,0.6);
       mecanumDrive.rightTurn(90,0.6);
       mecanumDrive.forward(24,0.6);
       //mecanumDrive.rightStrafe();


        telemetry.addData("DebugWait", "Auto Done.  Drive me home.");
        telemetry.update();
        while (opModeIsActive()) {

            double speed = 1;

            speed = (gamepad1.right_trigger * 0.6) + 0.4;
            double fwd = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            fwd = fwd * speed;
            strafe = strafe * speed * 1.6;
            if (strafe > 1) {
                strafe = 1;
            } else if (strafe < -1) {
                strafe = -1;
            }
            rot = rot * speed;
            mecanumDrive.setMotors(strafe, fwd, rot, 1);
        }

        /*  //START OF FFAuto
        // Auto Position 2 Fancy








        //dump
        mecanumDrive.backward(5,.5);
        mecanumDrive.rightTurn(45,.5);
        mecanumDrive.backward(24,.5);
        robot.setDoorPosition(FFRobot.doorPosition.DUMP);
        sleep(500);

        //duck wheel
        mecanumDrive.forward(6,.5);
        mecanumDrive.rightTurn(90,.5);
        mecanumDrive.forward(48,.5);

        for (int i=1; i<10; i++) {
            robot.setDuckWheel(0.1*i);
            sleep(50);
        }
        robot.setDuckWheel(1);
        sleep(3000);
        robot.setDuckWheel(0);

        //pickup
        mecanumDrive.backward(6,.5);
        mecanumDrive.rightTurn(135,.5);
        mecanumDrive.forward(96,.5);
        robot.pickup(true);
        mecanumDrive.forward(5,.5);
        robot.pickup(false);

        //dump
        mecanumDrive.leftTurn(45,.5);
        mecanumDrive.backward(48,.5);
        robot.setDoorPosition(FFRobot.doorPosition.DUMP);
        mecanumDrive.forward(48,.5);

        //Drive back - comment out for competition
        robot.moveArm(FFRobot.armPosition.PICKUP);
        sleep(300);
        robot.setDoorPosition(FFRobot.doorPosition.PICKUP);
        while (opModeIsActive()) {
            double speed = 1;

            speed = (gamepad1.right_trigger * 0.5) + 0.5;
            double fwd = addDeadZone(gamepad1.left_stick_y);
            double strafe = addDeadZone(gamepad1.left_stick_x);
            double rot= addDeadZone(gamepad1.right_stick_x);

            fwd = fwd * speed;
            strafe =strafe * speed * 1.6;
            if (strafe > 1) {
                strafe = 1;
            } else if (strafe < -1) {
                strafe = -1;
            }
            rot = rot * speed;
            mecanumDrive.setMotors(strafe,fwd,rot, 1);
        }




        /* [This is for the ring selection could use this for FF]
        while (!isStarted()) {
            UGObjectDetector.ringStackState ringsFound = OD.findRings();
            if (ringsFound == NONE) {
                telemetry.addLine("NONE Detected");
            } else if (ringsFound == SINGLE) {
                telemetry.addLine("SINGLE Detected");

            } else if (ringsFound == QUAD) {
                telemetry.addLine("QUAD Detected");

            } else {
                telemetry.addLine("This Cant Happen");
            }
            telemetry.update();
            this.idle();
        }

        waitForStart();

        if (opModeIsActive()) {
            UGObjectDetector.ringStackState ringsFound = OD.findRings();
            OD.shutdown();
*/

        /* THIS is where the code starts:
            mecanumDrive.forward(60, 0.7);
            mecanumDrive.rightTurn(15, 0.2);
            autoMultiShoot();

            if (ringsFound == UGObjectDetector.ringStackState.NONE) {
                mecanumDrive.arcMove(0, 67, 0.3, RIGHT, true, true);  //turn
                mecanumDrive.forward(40, 0.5);                                                     //wobble goal
                mecanumDrive.backward(5,0.5);
                mecanumDrive.arcMove(0, -90, 0.3, RIGHT, false, true); //turn back towards the 2nd wobble goal
                robot.moveWobbleArm(UGRobot.wobblePosition.MID);                                                 //set the wobble arm
                mecanumDrive.backward(42,0.7);                                                     //go to second wobble goal
                robot.wobbleServo(true);                                                                   //grip the wobble goal
                sleep(1200);
                robot.moveWobbleArm(UGRobot.wobblePosition.CARRY);                                                  //lift up the wobble goal
                mecanumDrive.leftTurn(166,0.4);                                                  //turn toward box
                mecanumDrive.backward(35,0.7);                                                     //go to box
                robot.moveWobbleArm(UGRobot.wobblePosition.DOWN);                                                 //lower wobble goal
                robot.wobbleServo(false);                                                                  //let go of wobble goal
                sleep(850);
                mecanumDrive.arcMove(2,95,0.5,RIGHT,true,true);      //park
                mecanumDrive.forward(28,0.5,true);
                mecanumDrive.leftTurn(100,0.65);
                mecanumDrive.backward(19,0.5);

            } else if (ringsFound == UGObjectDetector.ringStackState.SINGLE) {
                mecanumDrive.rightTurn(8, 0.4);                                              //turn
                mecanumDrive.forward(46, 0.5);                                                //wobble goal
                mecanumDrive.backward(24,0.5);
                mecanumDrive.arcMove(5,-58,0.4,RIGHT,false,true); //endStopped was false
                //mecanumDrive.arcMove(8,12,0.4,RIGHT,false,false);                                         X
                //mecanumDrive.rightTurn(35,0.5);                                              //pointed at ring and wobble goal
                robot.moveWobbleArm(UGRobot.wobblePosition.MID);                                            //set wobble arm up
                robot.setPickup(UGRobot.pickupDirection.IN);
                mecanumDrive.backward(45,0.4);                                                //pick up ring and go to wobble goal
                robot.wobbleServo(true);                                                              //grip wobble goal
                sleep(1200);
                robot.moveWobbleArm(UGRobot.wobblePosition.CARRY);                                             //pickup the wobble goal
                mecanumDrive.leftTurn(155,0.5);                                             //point at box
                mecanumDrive.backward(57,0.6);                                                //go to box
                robot.moveWobbleArm(UGRobot.wobblePosition.DOWN);                                            //lower wobble goal
                robot.wobbleServo(false);                                                             //place wobble goal
                sleep(1200);
                mecanumDrive.forward(8,0.3);                                                 //park
                robot.setPickup(UGRobot.pickupDirection.STOP);
                mecanumDrive.leftTurn(183,0.5);
                mecanumDrive.backward(16,.5);
                autoMultiShoot();
                mecanumDrive.arcMove(4,192,0.5,LEFT,true,true);
                mecanumDrive.backward(5,0.5);

            } else if (ringsFound == UGObjectDetector.ringStackState.QUAD) {
                mecanumDrive.rightTurn(16, 0.4);                                                  //turn
                mecanumDrive.forward(80, 0.7);                                                     //wobble goal
                mecanumDrive.backward(8,0.7);
                mecanumDrive.arcMove(0, -33, 0.3, RIGHT, false, true); //turn back towards the 2nd wobble goal
                robot.moveWobbleArm(UGRobot.wobblePosition.MID);                                                 //set the wobble arm
                mecanumDrive.backward(85,0.7);                                                     //go to second wobble goal
                robot.wobbleServo(true);                                                                   //grip the wobble goal
                sleep(1200);
                robot.moveWobbleArm(UGRobot.wobblePosition.CARRY);                                                  //lift up the wobble goal
                mecanumDrive.leftTurn(178,0.4);                                                  //turn toward box
                mecanumDrive.backward(78,0.7);                                                     //go to box
                robot.moveWobbleArm(UGRobot.wobblePosition.DOWN);                                                 //lower wobble goal
                robot.wobbleServo(false);                                                                  //let go of wobble goal
                sleep(850);
                mecanumDrive.forward(8,0.5);
                mecanumDrive.rightTurn(45,0.5);
                mecanumDrive.forward(29,0.5,true);
                mecanumDrive.rightTurn(135,0.5);
            }
        } else {
            OD.shutdown();
        }

*/

    }
    double addDeadZone(double input) {
        if (Math.abs(input) < 0.1) {return(0.0);}
        return(input);
    }
    /* Multi shot sequence:
    private void autoMultiShoot() {
        robot.multiShoot();
        while(robot.notDoneShooting()){
            robot.tick();
            mecanumDrive.tickSleep();
        }
    }*/


}
