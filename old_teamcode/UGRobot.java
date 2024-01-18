/*7078 mecanumm drive code */

package org.firstinspires.ftc.old_teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;

public class UGRobot implements MecanumDrive.TickCallback {

    private Telemetry telemetry;
    private LinearOpMode opMode;



    enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT}

    private DcMotor pickupbottom = null;
    private DcMotor pickuptop = null;
    private DcMotor pickup = null;
    FlywheelMC flyWheel = null;
    public DcMotor wobbleArmMotor = null;
    private Servo launchServo;
    private Servo gripper;
    public pickupDirection pickupState;
    public shooterDirection shooterState;
    private double flywheelPower = 0.72;
    private int upWobble = 1080;
    private int downWobble = 0;
    private int midWobble = 370;
    private int carryWobble = 610;
    private int lastPosition = 0;
    private double wobblePower = 0.75;
    private ArrayList<Long> toggleQueue = new ArrayList<Long>();
    private boolean launchServoState;
    int multishotDelay = 100;

    enum pickupDirection {IN, OUT, STOP}
    enum shooterDirection {OUT, STOP}
    enum wobblePosition {UP,DOWN,MID,CARRY}
    //enum wobbleDirection {UP,DOWN,STOP}


    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {

        telemetry = telemetryIn;
        opMode = opModeIn;

        launchServo = hardwareMap.get(Servo.class,"launchServo");
        gripper = hardwareMap.get(Servo.class,"gripper");

        pickupbottom = hardwareMap.get(DcMotor.class, "pickupBottom");
        pickuptop = hardwareMap.get(DcMotor.class, "pickupTop");
        wobbleArmMotor = hardwareMap.get(DcMotor.class,"wobble");
        wobbleArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheel = new FlywheelMC(hardwareMap,"shooter",600000);

        flyWheel.setDirection(DcMotor.Direction.REVERSE);
        flyWheel.setBackwardsEncoder(true);

        flyWheel.setHistorySize(3);

        pickupbottom.setPower(0);
        pickuptop.setPower(0);
        setFlywheel(shooterDirection.OUT);
        flyWheel.setPowerManual(flywheelPower);
        flyWheel.setPowerScale(150/100000000.0);
        flyWheel.setLookAheadTime(.35);
        setLaunchServo(false);


        lastPosition = wobbleArmMotor.getCurrentPosition();
        wobbleArmMotor.setPower(-0.2);

        telemetry.addData("last:", lastPosition);
        telemetry.addData("power:", wobbleArmMotor.getPower());
        telemetry.addData("timea",opMode.getRuntime());

        opMode.sleep(50);
        telemetry.addData("current:", wobbleArmMotor.getCurrentPosition());
        telemetry.addData("timeb",opMode.getRuntime());
        telemetry.update();

        while (wobbleArmMotor.getCurrentPosition() < lastPosition && !opMode.isStarted()) {
            lastPosition = wobbleArmMotor.getCurrentPosition();
            telemetry.addData("Encoder:", wobbleArmMotor.getCurrentPosition());
            telemetry.addData("power:", wobbleArmMotor.getPower());
            telemetry.update();
            opMode.sleep(50);
        }
        wobbleArmMotor.setPower(0);
        wobbleArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setMotorDirections (MecanumDrive mecanumDrive) {
        mecanumDrive.setMotorDirections(FORWARD, REVERSE, FORWARD, REVERSE);
    }

    public void tick () {
        long now = System.nanoTime();
        Long entry = null;
        for (Long when : toggleQueue) {
            if (now>when){
                entry = when;
            }
        }
        if (entry != null) {
            setLaunchServo(!launchServoState);
            toggleQueue.remove(entry);
        }
        flyWheel.setPowerAuto(flywheelPower);
    }

    public void tickCallback() {
        tick();
    }

    public void moveWobbleArm (wobblePosition targetPosition){
        if (targetPosition == wobblePosition.UP){
            wobbleArmMotor.setTargetPosition(upWobble);
            wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleArmMotor.setPower(wobblePower);
        } else if (targetPosition == wobblePosition.DOWN) {
            wobbleArmMotor.setTargetPosition(downWobble);
            wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleArmMotor.setPower(wobblePower);
        } else if (targetPosition == wobblePosition.MID){
            wobbleArmMotor.setTargetPosition(midWobble);
            wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleArmMotor.setPower(wobblePower);
        } else if (targetPosition == wobblePosition.CARRY){
            wobbleArmMotor.setTargetPosition(carryWobble);
            wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleArmMotor.setPower(wobblePower);
        }
    }

    public void wobbleServo (boolean close){
        if (close){
            gripper.setPosition(1);
        } else {
            gripper.setPosition(0);
        }
    }

    public void multiShoot (){
        clearQueue();
        setFlywheel(UGRobot.shooterDirection.OUT);
        setLaunchServo(true);
        for (int i=1;i<=3;i++) {
            addQueue(multishotDelay * i);
        }
        for (int i=5;i<=8;i++) {
            addQueue(multishotDelay * i);
        }

    }

    //public void addWobble(UGRobot.wobbleDirection direction) {
        //wobbleState = direction;
        //case UP:
    //}

    public void clearQueue (){
        toggleQueue.clear();
    }

    public boolean notDoneShooting() {
        return !toggleQueue.isEmpty();
    }

    public void addQueue (int whenMS) {
        toggleQueue.add(System.nanoTime()+(whenMS*1000000));
    }

    public void oldShoot() {
        setLaunchServo(true);
        opMode.sleep(200);
        setLaunchServo(false);
        opMode.sleep(200);

    }

    public void shoot () {
        clearQueue();
        setFlywheel(UGRobot.shooterDirection.OUT);
        setLaunchServo(true);
        addQueue(200);
    }

    public double findShooterSpeed () {
        flyWheel.updateCurrentVelocity();
        return (flyWheel.getCurrentVelocity());
    }

    public void setPickup(UGRobot.pickupDirection direction) {
        pickupState = direction;
        switch (direction) {
            case IN:
                pickupbottom.setPower(-1);
                pickuptop.setPower(-1);
                break;
            case OUT:
                pickupbottom.setPower(1);
                pickuptop.setPower(1);
                break;
            case STOP:
                pickupbottom.setPower(0);
                pickuptop.setPower(0);
                break;
        }
    }

    public void setLaunchServo (boolean in) {
        if(in) {
            launchServo.setPosition(0);
            launchServoState = true;
        } else {
            launchServo.setPosition(0.55);
            launchServoState = false;
        }
    }

    public double getFlywheelPower() {
        return flywheelPower;
    }

    public void setFlywheelPower(double flywheelPower) {
        this.flywheelPower = flywheelPower;
    }

    public int getShooterEncoderPosition() {
        return flyWheel.motor.getCurrentPosition();
    }

    public void setFlywheel(UGRobot.shooterDirection direction) {
        shooterState = direction;
        switch (direction) {
            case OUT:
                flyWheel.setPowerAuto(flywheelPower);
                break;
            case STOP:
                flyWheel.setPowerManual(0);
                break;
        }
    }
}