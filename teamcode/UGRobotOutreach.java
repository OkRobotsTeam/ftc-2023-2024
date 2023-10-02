/*7078 mecanumm drive code */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class UGRobotOutreach implements MecanumDrive.TickCallback {

    private Telemetry telemetry;
    private LinearOpMode opMode;
    private long flywheelSpinUpStart = 0;

    private long flywheelSpinUpDelayMax = 1500;

    private long shootingStartTime = 0;
    private boolean shootingStarted = false;

    private long shooterSpinDownDelay = 3000;


    enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT}

    private DcMotor pickupbottom = null;
    private DcMotor pickuptop = null;
    private DcMotor pickup = null;
    FlywheelMC flywheel = null;
    private Servo launchServo;
    public pickupDirection pickupState;
    public shooterDirection shooterState;
    private double flywheelPower = 0.72;
    private boolean flywheelOn = false;

    private int upWobble = 1080;
    private int downWobble = 0;
    private int midWobble = 370;
    private int carryWobble = 610;
    private int lastPosition = 0;
    private double wobblePower = 0.75;
    private ArrayList<Long> toggleQueue = new ArrayList<Long>();
    private boolean launchServoState;
    long multishotDelay = 100;

    enum pickupDirection {IN, OUT, STOP}
    enum shooterDirection {OUT, STOP}
    enum wobblePosition {UP,DOWN,MID,CARRY}
    //enum wobbleDirection {UP,DOWN,STOP}


    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {

        telemetry = telemetryIn;
        opMode = opModeIn;

        launchServo = hardwareMap.get(Servo.class,"launchServo");

        pickupbottom = hardwareMap.get(DcMotor.class, "pickupBottom");
        pickuptop = hardwareMap.get(DcMotor.class, "pickupTop");

        flywheel = new FlywheelMC(hardwareMap,"shooter",600000);

        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setBackwardsEncoder(true);

        flywheel.setHistorySize(3);

        pickupbottom.setPower(0);
        pickuptop.setPower(0);
        setFlywheel(shooterDirection.OUT);
        flywheel.setPowerManual(flywheelPower);
        flywheel.setPowerScale(150/100000000.0);
        flywheel.setLookAheadTime(.35);
        setLaunchServo(false);



        telemetry.addData("last:", lastPosition);
        telemetry.addData("timea",opMode.getRuntime());

        opMode.sleep(50);
        telemetry.addData("timeb",opMode.getRuntime());
        telemetry.update();


    }
    public void setMotorDirections (MecanumDrive mecanumDrive) {
        mecanumDrive.setMotorDirections(FORWARD, REVERSE, FORWARD, REVERSE);

    }

    boolean shooterAtSpeed() {
        if (!flywheelOn) return false;
        if (System.currentTimeMillis() > flywheelSpinUpStart + flywheelSpinUpDelayMax) {
            System.out.println("Spinup max time exceeded.  Returning true:" + flywheelSpinUpDelayMax + " + " + flywheelSpinUpDelayMax);
            return true;
        } else if (flywheel.getCurrentVelocity() > (flywheel.getDesiredVelocity() - 100)) {
            System.out.println("Spinup done.  RPM is now" + flywheel.getCurrentVelocity() + "of desired" + flywheel.getDesiredVelocity());
            return true;
        }
        return false;
    }

    public void tick () {
        long now = System.currentTimeMillis();
        if (shootingStarted) {
            if (now > (shootingStartTime + shooterSpinDownDelay) ) {
                flywheelOn = false;
                shootingStarted = false;
            }
        }
        if (flywheelOn) {
            flywheel.setPowerAuto(flywheelPower);
            if (!shootingStarted) {
                if (shooterAtSpeed()) {
                    shootingStarted = true;
                    shootingStartTime = now;
                }
            }
            if (shootingStarted) {
                Long entry = null;
                for (Long when : toggleQueue) {
                    if ( (now-shootingStartTime) >= when ) {
                        entry = when;
                    }
                }
                if (entry != null) {
                    setLaunchServo(!launchServoState);
                    toggleQueue.remove(entry);
                }
            }

        } else {
            flywheel.updateCurrentVelocity();
            flywheel.setPowerManual(0);
        }
    }

    public void tickCallback() {
        tick();
    }

    public void startShooting() {
        toggleQueue.clear();
        launchServoState=false;
        flywheelOn = true;
        flywheelSpinUpStart = System.currentTimeMillis();

        if (shootingStarted) {
            shootingStartTime = System.currentTimeMillis();
        }
    }


    public void shoot () {
        startShooting();
        addQueue(0);
        addQueue(200);
    }


    public void multiShoot (){
        startShooting();
        for (int i=0;i<=3;i++) {
            addQueue(multishotDelay * i );
        }
        for (int i=5;i<=8;i++) {
            addQueue(multishotDelay * i);
        }

    }

    public void addQueue (long whenMS) {
        toggleQueue.add(whenMS);
    }


    public double findShooterSpeed () {
        flywheel.updateCurrentVelocity();
        return (flywheel.getCurrentVelocity());
    }

    public void setPickup(UGRobotOutreach.pickupDirection direction) {
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
        return flywheel.motor.getCurrentPosition();
    }

    public void setFlywheel(UGRobotOutreach.shooterDirection direction) {
        shooterState = direction;
        switch (direction) {
            case OUT:
                flywheel.setPowerAuto(flywheelPower);
                break;
            case STOP:
                flywheel.setPowerManual(0);
                break;
        }
    }
}
