

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.FFRobot.armPosition;
import static org.firstinspires.ftc.teamcode.FFRobot.doorPosition;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleopFreightFrenzy", group="LinearOpMode")
@Disabled


public class TeleopFreightFrenzy extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private FFRobot robot = new FFRobot();

    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double duckWheelSpeed=0;

    @Override
    public void runOpMode() {
        mecanumDrive.init(hardwareMap, telemetry, this);


        robot.init(hardwareMap,telemetry,this);


        // Tell the driver that initialization is complete.


        // Wait for the game to start (driver presses PLAY)
        while (!isStarted() && !isStopRequested()) {

            telemetry.addData("Status", "Initialized");
            telemetry.addData("gp1lt", gamepad1.left_trigger);
           // telemetry.addData("Door Position", robot.pickupDoor.getPosition());
            //telemetry.addData("Left/Right Stick", "LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("Motor Power",robot.arm.getPower());
            telemetry.addData("Motor Position",robot.arm.getCurrentPosition());
            telemetry.addData("Dropped arm:", robot.dropped);

            telemetry.update();
            mecanumDrive.tickSleep();
        }
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        robot.setDoorPosition(doorPosition.CARRY);
        robot.moveArm(armPosition.CARRY);

        while (opModeIsActive()) {
            double speed = 1;

             speed = ( (gamepad1.right_trigger*gamepad1.right_trigger) * 0.7) + 0.3;
            double fwd = -addDeadZone(gamepad1.left_stick_y);
            double strafe = -addDeadZone(gamepad1.left_stick_x);
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

            if (gamepad2.a) {
                robot.moveArm(armPosition.PICKUP);
                robot.setDoorPosition(doorPosition.PICKUP);
                robot.pickup(true);
            } else if (gamepad2.b) {
                robot.setDoorPosition(doorPosition.DUMP);
                robot.pickup(false);
            } else {
                robot.pickup(false);
                if (robot.roughTargetPosition == armPosition.PICKUP) {
                    //robot.moveArm(armPosition.CARRY);
                    robot.moveArm(armPosition.CARRY, (int) (gamepad2.left_trigger*250) ) ;

                } else if (robot.roughTargetPosition == armPosition.HIGH) {
                    robot.moveArm(armPosition.HIGH, (int) (gamepad2.left_trigger*250) ) ;
                }
                if (gamepad2.dpad_up) {
                    robot.setDoorPosition(doorPosition.CARRY);
                    robot.moveArm(armPosition.HIGH, (int) (gamepad2.left_trigger*250) ) ;
                } else if (gamepad2.dpad_down) {
                    robot.moveArm(armPosition.PICKUP);
                    robot.setDoorPosition(doorPosition.PICKUP);
                } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                    robot.setDoorPosition(doorPosition.CARRY);
                    robot.moveArm(armPosition.CARRY, (int) (gamepad2.left_trigger*250) ) ;
                } else {
                    robot.setDoorPosition(doorPosition.CARRY);
                    robot.moveArm(robot.roughTargetPosition, (int) (gamepad2.left_trigger*250) ) ;

                }
            }

            if (gamepad2.left_bumper) {
                duckWheelSpeed=duckWheelSpeed+0.1;
                if (duckWheelSpeed > 1) {duckWheelSpeed = 1;}
                robot.setDuckWheel(duckWheelSpeed);
            } else if(gamepad2.right_bumper) {
                duckWheelSpeed=duckWheelSpeed-0.1;
                if (duckWheelSpeed < -1) {duckWheelSpeed = -1;}
                robot.setDuckWheel(duckWheelSpeed);
                robot.setDuckWheel(duckWheelSpeed);
            } else {
                duckWheelSpeed=0;
                robot.setDuckWheel(duckWheelSpeed);
            }


            //robot.setDuckWheel(gamepad2.right_stick_x);
            //,.robot.setShippingElementPickupPosition(gamepad1.left_trigger);
            robot.setShippingElementPickupPosition(gamepad2.right_trigger);



//Idk if this part is neccesary
            telemetry.addData("gp1lt", gamepad2.left_trigger);

            telemetry.addData("gp1lt", gamepad1.left_trigger);
            //telemetry.addData("Left/Right Stick", "LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("Motor Power",robot.arm.getPower());
            telemetry.addData("Motor Position",robot.arm.getCurrentPosition());
            telemetry.update();
            mecanumDrive.tickSleep();
        }

    }
    double addDeadZone(double input) {
        if (Math.abs(input) < 0.1) {return(0.0);}
        return(input);
    }
}
