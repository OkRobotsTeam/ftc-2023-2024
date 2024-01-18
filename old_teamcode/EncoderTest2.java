

package org.firstinspires.ftc.old_teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HPMC;


@TeleOp(name="Encoder Wobble", group="Test")
@Disabled

public class EncoderTest2 extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motor;
    HPMC shooter;
    long nextWake;
    long tickTime = 50;
    private DcMotor wobbleArmMotor;

    @Override
    public void runOpMode() {

        wobbleArmMotor = hardwareMap.get(DcMotor.class,"wobble");
        wobbleArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wobbleArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleArmMotor.setPower(-0.2);
        int lastEncoder;
        int nowEncoder;
        int diff;
        lastEncoder = wobbleArmMotor.getCurrentPosition();
        sleep(50);
        nowEncoder = wobbleArmMotor.getCurrentPosition();
        diff = Math.abs(nowEncoder - lastEncoder);
        while(diff > 2 && !isStarted()) {
                lastEncoder = nowEncoder;
                telemetry.addData("Encoder:", wobbleArmMotor.getCurrentPosition());
                telemetry.addData("power:", wobbleArmMotor.getPower());
                telemetry.update();
                sleep(50);
                nowEncoder = wobbleArmMotor.getCurrentPosition();
                diff = Math.abs(nowEncoder - lastEncoder);
        }
        wobbleArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        wobbleArmMotor.setPower(0);

        waitForStart();
         while (opModeIsActive()) {
             wobbleArmMotor.setTargetPosition(500);
             wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


             wobbleArmMotor.setPower(0.5);
            telemetry.addData("Time:", runtime.seconds());
             telemetry.addData("Encoder:", wobbleArmMotor.getCurrentPosition());
             telemetry.addData("power:", wobbleArmMotor.getPower());
             telemetry.update();
             tickSleep();
         }
         wobbleArmMotor.setTargetPosition(0);
         wobbleArmMotor.setPower(1);
         sleep(500);

    }


    public void tickSleep() {

        long now = System.nanoTime();
        nextWake = nextWake + tickTime * 1000000;
        if (nextWake < now) {
            nextWake = now + tickTime * 1000000;
            double msLate = (now - nextWake) / 1000000;
            if (msLate > 100) {
                System.out.println(String.format("Either this is the first tick or something is wrong: %.1f ms late", msLate));
                throw new RuntimeException(String.format("Can't keep up: tick took %.1f ms too long", msLate));
            }
            return;
        }
        long sleepTime = (int) Math.floor((nextWake - now) / 1000000);
        //System.out.println("Sleeping: " + sleepTime);
        sleep(sleepTime);

    }
}
