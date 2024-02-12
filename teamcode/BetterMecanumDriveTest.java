package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.MathUtil.interpolate2D;


@TeleOp(name = "Test: Better Mecanum Drivetrain", group = "Test")

public class BetterMecanumDriveTest extends LinearOpMode {

    private final BetterMecanumDrive mecanumDrive = new BetterMecanumDrive(hardwareMap, null, this);

    @Override
    public void runOpMode() {
        mecanumDrive.setMotorDirections(new DcMotorSimple.Direction[]{FORWARD, REVERSE, FORWARD, REVERSE});

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Wait for the driver to start the OpMode
        waitForStart();

        while (opModeIsActive()) {

            double movementSpeedModifier = interpolate2D(
                    0, CSConstants.Drivetrain.minimumMovementPower,
                    1, CSConstants.Drivetrain.maximumMovementPower,
                    gamepad1.right_trigger);

            double turnSpeedModifier = interpolate2D(
                    0, CSConstants.Drivetrain.minimumTurnPower,
                    1, CSConstants.Drivetrain.maximumTurnPower,
                    gamepad1.right_trigger);

            mecanumDrive.moveWithController(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    movementSpeedModifier,
                    turnSpeedModifier,
                    CSConstants.Drivetrain.isFieldCentricDuringTeleOp
            );


            telemetry.addData("Left/Right Stick", "LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            mecanumDrive.telemetryMotorPower();
            mecanumDrive.telemetryMotorPosition();


            telemetry.update();
        }
    }
}
