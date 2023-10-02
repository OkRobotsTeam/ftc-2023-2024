package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;

/*

Braking Distance Motor Controller
The idea is simple and could probably be implemented in a much
less messy way, but I'm me and this is how I code.  -Eric

 */

public class BrakingDistanceMotorController {
    static final float NANOSECONDS_PER_SECOND = 1000000000;
    static final double LOOK_AHEAD_TIME = 0.075;
    static final double FINE_POWER_SCALE =  0.0002;
    static final long MS_PER_NS = 1000000;


    int historySize = 3;
    static long tickMillis = 50; //in milliseconds
    double tickSeconds = tickMillis / 1000.0;
    public int currentPosition = 0;
    private double desiredVelocity = 0;
    double currentVelocity = 0;  //in count per second
    double power = 0;
    double acceleration = 0;
    float updatesPerSecond = 0;
    double velocitySoon = 0;
    double change = 0;
    double maxSpeed = 0;

    long lastUpdateTime = 0;
    DcMotorEx motor;

    //The name of the motor.  Setting this enables stdout logging for debugging;
    String label = null;
    private double smAccelerationStartPower = 0.4;
    private int accelerationTicks = 4;
    private int decelerationTicks = 12;



    public enum MoveState { ACCELERATING, AT_SPEED, STOPPING, DONE, BRAKING}
    public enum Direction {FORWARD, REVERSE}

    //smooth move variables
    MoveState smState = MoveState.DONE;
    long smAccelerationTick;
    long smStartPosition;
    long smTargetPosition;
    double smStartSpeed;
    double smVelocity;   //speed with sign for direction
    private double smPower;  //unsigned.
    long  smDistance;
    long smStartStopping;
    long smDesiredPosition;
    int smDecelerationTick;
    boolean smEndStopped = true;
    long smSign = 1;
    long brakingSpeed = 500; //in count per second squared





    ArrayList<Integer> positionList = new ArrayList<>();
    ArrayList<Long> timeList = new ArrayList<>();

    public BrakingDistanceMotorController(DcMotorEx setMotor) {
        motor = setMotor;
        init();
    }
    public BrakingDistanceMotorController(DcMotorEx motorIn, double maxSpeedIn) {
        this(motorIn);
        maxSpeed = maxSpeedIn;
    }

    public BrakingDistanceMotorController(HardwareMap hardwareMap, String motorString) {
        this(hardwareMap.get(DcMotorEx.class, motorString));
        MotorConfigurationType motorType = motor.getMotorType();
        maxSpeed = motorType.getMaxRPM();
    }

    public BrakingDistanceMotorController(HardwareMap hardwareMap, String motorString, DcMotor.Direction direction)  {
        this(hardwareMap, motorString);
        setDirection(direction);
    }

    public BrakingDistanceMotorController(HardwareMap hardwareMap, String motorString, double maxSpeedIn) {
        this(hardwareMap.get(DcMotorEx.class, motorString));
        maxSpeed = maxSpeedIn;
    }

    public void init() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setAccelerationTicks(int ticks) {
        if (ticks < 1) {ticks = 1;}
        accelerationTicks = ticks;}
    public int getAccelerationTicks() {return accelerationTicks;}
    public void setDecelerationTicks(int ticks) {decelerationTicks = ticks;}
    public int getDecelerationTicks() {return decelerationTicks;}

    public void setAccelerationStartPower(double aspIn) { smAccelerationStartPower = aspIn; }
    public double getAccelerationStartPower() { return smAccelerationStartPower; }

    public void setBrakingSpeed(long countPerSecondSquared) { brakingSpeed = countPerSecondSquared;}
    public long getBrakingSpeed(){return brakingSpeed;}
    public void setLabel(String string) { label = string;}
    public void setHistorySize(int size) { historySize = size;}
    public double getPower() { return power; }
    public double getCurrentVelocity() { return currentVelocity; }
    public double getDesiredVelocity() { return desiredVelocity;}
    public double getAcceleration()  { return acceleration; }
    public double getVelocitySoon()  { return velocitySoon; }
    public double getUpdatesPerSecond() { return updatesPerSecond;}
    public int getCurrentPositionCached() { return currentPosition; }
    public int getTargetPosition() { return (int) smTargetPosition;}
    public int getCurrentPosition() { return motor.getCurrentPosition(); }

    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
        updateCurrentVelocity();
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    //takes a power, calculates a desired velocity and autoAdjusts to it
    public void setPowerAuto(double powerFactor) {
        if (powerFactor > 1) powerFactor = 1;
        if (powerFactor < -1) powerFactor = -1;
        desiredVelocity = maxSpeed * powerFactor;
        updateCurrentVelocity();
        autoAdjust();
    }

    public void setPower(double powerIn) {setPowerManual(powerIn);}
    public void setPowerManual(double powerIn) {
        power = powerIn;
        if (power > 1) { power = 1;}
        if (power < -1) { power = -1;}
        motor.setPower(power);
    }

    public void runToPosition(int position, double power) {
        int distance = position - motor.getCurrentPosition();
        System.out.println("RTP Called.  aticks " + accelerationTicks);
        smoothMoveSetup(distance,power, Direction.FORWARD,true);
    }
    public void runToPosition(int position, double power, int accelerationTicksIn, int decelerationTicksIn) {
        accelerationTicks = accelerationTicksIn;
        decelerationTicks = decelerationTicksIn;
        runToPosition(position, power);

    }

    public void setDesiredVelocity(double newDesiredVelocity) {
        desiredVelocity = newDesiredVelocity;
        updateCurrentVelocity();
        autoAdjust();
    }

    public void updateCurrentVelocity() {
        long nanotime = System.nanoTime();
        //Don't update if it's less than 10 ms since the last update
        if ( (nanotime - lastUpdateTime) <   (10 * MS_PER_NS) ) {
            debug("Skipping double update");
            return;
        } else if (nanotime - lastUpdateTime > (100* MS_PER_NS) ) {
            debug("currentPosition and time lists are outdated.  Clearing: " + (nanotime-lastUpdateTime)/1000000.0);
            positionList.clear();
            timeList.clear();
        }
        currentPosition = motor.getCurrentPosition();
        positionList.add(currentPosition);
        timeList.add(nanotime);
        lastUpdateTime = nanotime;
        if (positionList.size() > historySize) {
            positionList.remove(0);
        }
        if (timeList.size() > historySize) {
            timeList.remove(0);
        }
        //set currentVelocity to
        if (timeList.size() > 2) {
            int start = 0;
            int end = timeList.size() - 1;
            int middle = end / 2;

            long elapsed = timeList.get(end) - timeList.get(start);
            int distance = positionList.get(end) - positionList.get(start);
            long oldDistance = positionList.get(middle) - positionList.get(start);
            long newDistance = positionList.get(end) - positionList.get(middle);
            long oldElapsed = timeList.get(middle) - timeList.get(start);
            long newElapsed = timeList.get(end) - timeList.get(middle);
            if ((elapsed > 0) && (oldElapsed > 0) && (newElapsed > 0)) {
                currentVelocity = distance * (NANOSECONDS_PER_SECOND / elapsed);
                updatesPerSecond = (timeList.size() * (NANOSECONDS_PER_SECOND / elapsed));
                double oldSpeed = oldDistance * (NANOSECONDS_PER_SECOND / oldElapsed);
                double newSpeed = newDistance * (NANOSECONDS_PER_SECOND / newElapsed);
                double secondsElapsed = (newElapsed / NANOSECONDS_PER_SECOND);

                acceleration = (newSpeed - oldSpeed) / secondsElapsed;
            } else {
                debug("Not enough history.  Using zeros");
                currentVelocity = 0;
                acceleration = 0;
            }
        } else {
            currentVelocity = 0;
            acceleration =0;
        }
    }


    public void manualAdjust(double velocity) {
        power = velocity / maxSpeed;
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        motor.setPower(power);
    }


    //Set motor power based on desiredVelocity

    private void autoAdjust() {

        velocitySoon = currentVelocity + (acceleration * LOOK_AHEAD_TIME);
        /*if (braking) {
            //reset power to a sane value so if we no longer need to brake, the adjustments have a reasonable starting point
            power = currentVelocity / maxSpeed;
            braking = false;
        }*/
        double difference =  desiredVelocity - velocitySoon;
        if ( Math.abs(desiredVelocity) < 1) {
            //we want to be stopped.  Zero the power.
            change = -power;
            //System.out.println(String.format("Change from zeroing: %.4f", change));
        //} else if ( (velocitySoon / desiredVelocity) > 1.2 && Math.abs(difference) > 400 ) {
        //    power = recoverLostSign(0.15, smVelocity);
        //    braking = true;
            //System.out.println(String.format("Change from overspeed: %.4f", change));
        } else if ( (velocitySoon / desiredVelocity) > 1.01) {
            change =  (power * (desiredVelocity / velocitySoon)) - power;
            //System.out.println(String.format("Change from overspeed: %.4f", change));
        } else {
            change = FINE_POWER_SCALE * difference;
            //System.out.println(String.format("Change from power scale: %.4f", change));
        }

        /*
        if (Math.abs(change) > MAX_POWER_CHANGE) {
            if (change < 0) {
                change = -MAX_POWER_CHANGE;
            } else {
                change = MAX_POWER_CHANGE;
            }
        }*/

        power = power + change;
        //debug(String.format("Power: %.4f change: %.4f velocitySoon: %.2f  desiredVelocity: %.2f diff: %.2f pos: %d", power, change, velocitySoon, desiredVelocity, difference,currentPosition));
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        double motorPower = 0;
        if (Math.abs(desiredVelocity)  > 10 || ( Math.abs(power) > 0.003 ) ) {
            motorPower = power * 0.8 + recoverLostSign(0.15f, power);
        }
        debug(String.format("Power: %.3f MotorPower: %.3f", power, motorPower));
        motor.setPower(motorPower);
    }


    private void smoothMoveSetup(long distance, double power, Direction direction, boolean endStopped) {
        updateCurrentVelocity();
        smStartPosition = currentPosition;

        if (distance < 0) {
            //reverse direction
            switch (direction) {
                case FORWARD:
                    direction = Direction.REVERSE;
                    break;
                case REVERSE:
                    direction = Direction.FORWARD;
            }
        }

        smDistance = (long) Math.abs(distance);

        if (direction == Direction.FORWARD) {
            smTargetPosition = smStartPosition + smDistance;
        } else {
            smTargetPosition = smStartPosition - smDistance;
        }

        desiredVelocity = currentVelocity;
        smStartSpeed = currentVelocity;
        smAccelerationTick = 0;
        //smDesiredProgress = 0;
        smDesiredPosition = currentPosition;
        smDecelerationTick = 0;
        smEndStopped = endStopped;

        if (direction == Direction.FORWARD) {
            smVelocity = power*maxSpeed;
            smSign = 1;
            smPower = power;
        } else {
            smVelocity = -power*maxSpeed;
            smSign = -1;
            smPower = power;
        }

        smState = MoveState.ACCELERATING;
        double  distanceDecelerating = (smVelocity / 2 ) * ((accelerationTicks+1) * tickMillis / 1000.0);
        if (endStopped) {
            smStartStopping = (long) (smDistance - Math.abs(distanceDecelerating));
            if (smStartStopping < (smDistance *0.4) ) {
                smStartStopping = (long) (smDistance *0.4);
            }
        } else {
            smStartStopping = smDistance;
        }
        debug2(String.format("Setting Up SM:  Distance: %d Power: %.2f  AT: %d D: %s SD %.2f   Start Slowing At:  %d", smDistance, power, accelerationTicks, direction.toString(), distanceDecelerating, smStartStopping));

    }

    public double percentComplete() {
        return ( moved() / (double) smDistance);
    }


    public String getMoveState() {
        String percent;
        if (smDistance > 0) {
            percent = String.format(" %.1f%% %d",  (double) moved()*100 / (double) smDistance, smDistance - moved() );

            percent = String.format(" %.1f%% %d",  (double) moved()*100 / (double) smDistance, distanceLeft() );

        } else {
            percent = "---";
        }
        if (smState == MoveState.ACCELERATING) {
            return  percent + "-A";
        } else if (smState == MoveState.AT_SPEED) {
            return  percent + "-R";
        } else if ( smState == MoveState.BRAKING) {
            return  percent + "-B";
        } else if ( smState == MoveState.STOPPING) {
            return  percent + "-S" + (decelerationTicks - smDecelerationTick);
        } else if ( smState == MoveState.DONE) {
            return  percent + "-D";
        } else {
            return percent + "-?";
        }
    }

    //returns true if we're not done moving
    public boolean smTick() {
        return smTick(0);
    }

    public boolean smTick(double targetPercentDone) {
        double catchupVelocity = 0;
        //If we will reach the end of our travel before we tick again, stop now.
        if ((smState != MoveState.DONE) && (distanceLeft()  <  distancePerTick(Math.abs(currentVelocity)/ 2) ) || ( distanceLeft() < 3) ) {
            debug2(String.format("DONE  Moved: %d of %d", moved(), smDistance));
            stopIfEndingStopped();
            smState = MoveState.DONE;
            return(false);
        }

        double catchupAdjustment = 0;
        if (targetPercentDone > 0) {
            double targetPosition = (smTargetPosition - smStartPosition) * targetPercentDone + smStartPosition;
            double catchupDistance = targetPosition - currentPosition;
            //try to catch up 1/2 of the remaining distance this tick;
            catchupVelocity = catchupDistance / (tickSeconds * 2);
            debug(String.format("Catchup Data:  TPD: %.4f  targetPosition: %.1f  currentPosition: %d  catchupDistance: %.4f, catchupVelocity: %.1f", targetPercentDone, targetPosition, currentPosition ,catchupDistance, catchupVelocity));
            //catchupVelocity = 0;   //Yea.  I had bugs.
            //desiredVelocity = smVelocity + catchupVelocity;
            catchupAdjustment = catchupVelocity/maxSpeed;
        }
        //debug(String.format("Tick starting.  Moved %d of %d.  State: %s", moved(), smDistance, smState.toString() ));
        long stoppingDistanceLeft = distanceLeft();

        switch (smState) {
            case ACCELERATING:
                smAccelerationTick++;
                if (smAccelerationTick == 1) {
                    //set power to the specified starting point.
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    power = smAccelerationStartPower;
                } else {
                    double additionalPowerThisTick = ((smPower - smAccelerationStartPower) * (smAccelerationTick - 1 / accelerationTicks));
                    power = additionalPowerThisTick + smAccelerationStartPower + catchupAdjustment;
                }
                power = power*smSign;
                double currentPercentVelocity = currentVelocity/maxSpeed;
                debug(String.format("ACC Spd: %.1f%% (%.1f of %.1f) Moved: %d Pow:%.1f", currentPercentVelocity * 100.0, currentVelocity, maxSpeed, moved(), power));
                setPowerManual(power);

                //check if it's time to be done accelerating
               if ( (currentVelocity*decelerationTicks*tickSeconds) > (distanceLeft()*2) ) {
                   //End Accelerating if we're so close that if we start decelerating now we'll overshoot the end.
                   debug(String.format("We're too close to the end to keep accelerating", moved(), smDistance, currentVelocity, smVelocity));
                   smState = MoveState.AT_SPEED;
               } else if ((moved() > (smDistance / 3.0)) || ((currentVelocity / smVelocity) > 0.9)) {
                   //End Accelerating if we're more than 1/3rd of the way there, or if we're up to speed.
                   debug(String.format("Switching to AT_SPEED  (%d > %d / 3) or ( %.1f / %.1f) > 0.9", moved(), smDistance, currentVelocity, smVelocity));
                   smState = MoveState.AT_SPEED;
               } else if (moved() > smStartStopping) {
                    if (smEndStopped) {
                        smState = MoveState.STOPPING;
                    } else {
                        smState = MoveState.DONE;
                        return (false);
                    }
                } else if (smAccelerationTick >= accelerationTicks) {
                    smState = MoveState.AT_SPEED;
                }
                //debug(String.format("ACC %.2f%% DS: %.2f  CS: %.2f Moved: %d(%d) of %d PWR:%.2f", targetPercentSpeed, desiredVelocity, currentVelocity, moved, smDesiredProgress, smStartStopping, power));
                return (true);
            case AT_SPEED:

                //switching to simple adjustment based on the power of the move, not based on maintaining a particular speed.
                power = (smPower + catchupAdjustment)*smSign;
                setPowerManual(power);

                //check to see if it's time to switch states
                long decelerationDistance = (currentDistancePerTick() * (decelerationTicks + 2) / 2) ;
                debug(String.format("A_S DS:%.2f  CS:%.2f Moved:%d of %d PWR:%.2f DpT: %d DL:%d BD:%d", desiredVelocity, currentVelocity, moved(), smStartStopping, power, currentDistancePerTick(), distanceLeft(), decelerationDistance));
                if (smEndStopped &&  (Math.abs(distanceLeft()) < decelerationDistance )) {
                    if (smEndStopped) {
                        debug("Switching to Stopping because deceleration ramp time needs us to");
                        smState = MoveState.STOPPING;
                    } else {
                        smState = MoveState.DONE;
                        return(false);
                    }
                }
                return(true);
            case STOPPING: {
                smDecelerationTick++;
                long ticksLeft = decelerationTicks - smDecelerationTick - 1;
                if (ticksLeft < 1 ) {
                    stopIfEndingStopped();
                    smState = MoveState.DONE;
                    return (false);
                }
                //double ratio = (0.7 * smDecelerationTick  / (double) smDecelerationTicks) + 0.1;
                //desiredVelocity = (smSpeed  - (smSpeed * ratio));


                double timeLeft = ( (ticksLeft+1)  * tickSeconds);
                desiredVelocity = (stoppingDistanceLeft / timeLeft * 2.3 * smSign);
                desiredVelocity = desiredVelocity + catchupVelocity;

                //estimated braking speed change is 500/50ms;
                double minStoppingTicks = Math.abs(currentVelocity / brakingSpeed / tickSeconds);  //seconds to stop times half of our current speed
                long minStoppingDistance = (long) ( Math.abs(currentVelocity) * minStoppingTicks * tickSeconds)/2;
                if (Math.abs(currentVelocity) < Math.abs(desiredVelocity)*1.5) {
                    //crude fast adjustment should be best here
                    power=(desiredVelocity / maxSpeed* 0.8) + (0.2*smSign);
                    setPowerManual(power);
                    debug(String.format("GLI DSPD: %.2f  SPD: %.2f Left: %d MSD:%d TL:%d M:%d of %d PWR:%.2f", desiredVelocity, currentVelocity, stoppingDistanceLeft, minStoppingDistance,  ticksLeft, moved(), smDistance, power));
                    return (true);
                } else if (stoppingDistanceLeft > minStoppingDistance) {
                    //coast using minimum power before braking kicks in
                    power = 0.11*smSign;
                    setPowerManual(power);
                    debug(String.format("SLO DSPD: %.2f  SPD: %.2f Left: %d MSD:%d TL:%d M:%d of %d PWR:%.2f", desiredVelocity, currentVelocity, stoppingDistanceLeft, minStoppingDistance,  ticksLeft, moved(), smDistance, power));
                    return (true);
                } else {
                    debug(String.format("BRK DSPD: %.2f  SPD: %.2f Left: %d MSD:%d TL:%d M:%d of %d PWR:%.2f", desiredVelocity, currentVelocity, stoppingDistanceLeft, minStoppingDistance, ticksLeft, moved(), smDistance, power));
                    stopIfEndingStopped();
                    return (true);
                }
            }
            case DONE:
                //debug(String.format("DONE DSPD: %.2f  SPD: %.2f M:%d of %d PWR:%.2f", desiredVelocity, currentVelocity,  moved(), smDistance, power));
                stopIfEndingStopped();
                return(false);
        }
        return false;
    }

    boolean onTick() {
        updateCurrentVelocity();
        if (smState != MoveState.DONE) {
            return smTick();
        } else {
            return false;
        }
    }
    double recoverLostSign(double number, double source) {
        if (source<0) {
            return(-number);
        }
        return number;
    }

    void stopIfEndingStopped() {
        if (smEndStopped) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            power = 0;
            motor.setPower(0);
            desiredVelocity = 0;
        }
    }
    public void stop() {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            power = 0;
            motor.setPower(0);
            desiredVelocity = 0;
            smState= MoveState.DONE;
    }

    public void setTickTime(long tickTimeIn) {
        tickMillis = tickTimeIn;
        double tickSeconds = tickMillis / 1000.0;
    }



    public String getSMStatus() {
        return String.format("State: %s  Moved: %d Speed: %.2f  Desired: %.2f Power: %.2f StoppingAt: %d",
                smState.toString(),
                Math.abs(currentPosition - smStartPosition),
                currentVelocity, desiredVelocity, power, smStartStopping);
    }

    void debug(String string) {
        if (label != null) {
            System.out.println("M: " + label + " " + string);
        }
    }
    void debug2(String string) {
        if (label != null) {
            System.out.println("M: " + label + " " + string);
        }
    }


    long moved() {
        if (smTargetPosition < smStartPosition) {
            return(smStartPosition - currentPosition);
        } else {
            return(currentPosition - smStartPosition);

        }
    }

    long distanceLeft() {
        return smDistance - moved();
    }

    public int smGetMovementError() {
        return (int) Math.abs(distanceLeft());
    }

    long distancePerTick(double velocity) {
        return (long) (velocity * tickSeconds);
    }
    long currentDistancePerTick() {
        return  Math.abs( (long) (currentVelocity * tickSeconds));
    }
    long movedPerTick(double velocity) {
        return Math.abs(distancePerTick(velocity));
    }
}




