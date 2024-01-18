

package org.firstinspires.ftc.old_teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.old_teamcode.UGRobotOutreach.pickupDirection;
import static org.firstinspires.ftc.old_teamcode.UGRobotOutreach.shooterDirection;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp(name="Ring Shooter", group="Outreach")

public class UGOutreach extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private boolean isSpeedUpPressed = false;
    private boolean isSpeedDownPressed = false;
    private boolean isShootTriggered = false;
    private boolean isWobbleUpTriggered = false;
    private boolean isWobbleDownTriggered = false;
    private boolean isWobbleOpenTriggered = false;
    private boolean isWobbleCloseTriggered = false;
    private boolean isMultiShootTriggered = false;


    private MecanumDrive mecanumDrive = new MecanumDrive();
    private UGRobotOutreach robot = new UGRobotOutreach();

    @Override
    public void runOpMode() {
        mecanumDrive.init(hardwareMap, telemetry, this);
        robot.setMotorDirections(mecanumDrive);

        robot.init(hardwareMap,telemetry,this);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double speed = 1;

             speed = (gamepad1.right_trigger * 0.6) + 0.4;
            double fwd = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rot= gamepad1.right_stick_x;

            fwd = fwd * speed;
            strafe =strafe * speed * 1.6;
            if (strafe > 1) {
                strafe = 1;
            } else if (strafe < -1) {
                strafe = -1;
            }
            rot = rot * speed;
            mecanumDrive.setMotors(strafe,fwd,rot, 1);

            boolean pull = gamepad2.x;
            boolean push = gamepad2.b;
            if (pull) {
                robot.setPickup(pickupDirection.IN);
                telemetry.addData("Manipulator Motors", "Pulling");
            } else if (push) {
                robot.setPickup(pickupDirection.OUT);
                telemetry.addData("Manipulator Motors", "Pushing");
            } else {
                telemetry.addData("Manipulator Motors", "Idle");
                robot.setPickup(pickupDirection.STOP);
            }


            if (gamepad2.left_bumper != isMultiShootTriggered) {
                //mecanumDrive.setMotors(0,0,0, 1);
                if (gamepad2.left_bumper) {
                    robot.multiShoot();
                }
                isMultiShootTriggered = gamepad2.left_bumper;
            }


            if (gamepad2.right_bumper != isShootTriggered) {
                //mecanumDrive.setMotors(0,0,0, 1);
                if (gamepad2.right_bumper) {
                    robot.shoot();
                }
                isShootTriggered = gamepad2.right_bumper;
            }

            if (gamepad2.start != isSpeedUpPressed) {
                if (gamepad2.start) {
                    robot.setFlywheelPower(robot.getFlywheelPower()+0.02);
                    robot.setFlywheel(shooterDirection.OUT);
                }
                isSpeedUpPressed = gamepad2.start;
            }

            if (gamepad2.back != isSpeedDownPressed) {
                if (gamepad2.back) {
                    robot.setFlywheelPower(robot.getFlywheelPower()-0.02);
                    robot.setFlywheel(shooterDirection.OUT);
                }
                isSpeedDownPressed = gamepad2.back;
            }





     

            mecanumDrive.tickSleep();
            robot.tick();
            //telemetry.addData("Left/Right Stick", "LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("Shoot Power", robot.getFlywheelPower());

            telemetry.addData("Current Velocity",robot.flywheel.getCurrentVelocity());
            telemetry.addData("Desired Velocity", robot.flywheel.getDesiredVelocity());
            telemetry.addData("MotorPower",robot.flywheel.motor.getPower());
            telemetry.addData("change",robot.flywheel.change);
            telemetry.addData("vs",robot.flywheel.difference);

            telemetry.update();
        }

    }

}
