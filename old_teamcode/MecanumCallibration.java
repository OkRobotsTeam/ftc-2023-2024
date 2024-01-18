

package org.firstinspires.ftc.old_teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name="Test: Mecanum Callibration", group="Test")
@Disabled
public class MecanumCallibration extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();


    private MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void runOpMode() {
        mecanumDrive.init(hardwareMap, telemetry, this);
        mecanumDrive.setMotorDirections(Direction.FORWARD, Direction.REVERSE, Direction.FORWARD, Direction.REVERSE);





        telemetry.addData("Status", "Initialized");


        waitForStart();
        runtime.reset();


        int j=100;
        while (opModeIsActive()) {

            double speed = 1;

             speed = (gamepad1.right_trigger * 0.6) + 0.4;
            if (gamepad1.dpad_up) {
                mecanumDrive.forward(24,speed);
            } else if (gamepad1.dpad_down){
                mecanumDrive.backward(24,speed);
            } else if (gamepad1.dpad_left){
                mecanumDrive.leftStrafe(24,speed);
            } else if (gamepad1.dpad_right){
                mecanumDrive.rightStrafe(24,speed);
            } else if (gamepad1.a){
                mecanumDrive.leftTurn(90,speed);
            } else if (gamepad1.b){
                mecanumDrive.rightTurn(90,speed);
            } else if (gamepad1.x){
                mecanumDrive.leftTurn(180,speed);
            } else if (gamepad1.y){
                mecanumDrive.rightTurn(180,speed);
            } else {
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

                if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
                    if (gamepad1.a) {
                        mecanumDrive.runMotors(0.5,0,0,0,false);
                        telemetry.addData("Running", "Front Left");
                    } else if (gamepad1.b) {
                        mecanumDrive.runMotors(0,0.5,0,0,false);
                        telemetry.addData("Running", "Front Right");
                    } else if (gamepad1.x) {

                        mecanumDrive.runMotors(0,0,0.5,0,false);
                        telemetry.addData("Running", "Back Left");
                    } else if (gamepad1.y) {
                        mecanumDrive.runMotors(0,0,0,0.5,false);
                        telemetry.addData("Running", "Back Right");
                    }
                } else {
                    mecanumDrive.setMotors(strafe, fwd, rot, 1);
                }

                telemetry.addData("Left/Right Stick", "LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
                mecanumDrive.telemetryMotorPower();
                mecanumDrive.telemetryMotorPosition();

            }


            j++;
            telemetry.addData("Number:", "%d", j);
            telemetry.update();
            mecanumDrive.tickSleep();
        }

    }

}
