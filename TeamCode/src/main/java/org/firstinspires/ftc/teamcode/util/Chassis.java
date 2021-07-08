package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Settings.Alliance;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Objects;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.util.MathFunction.getFollowPosition;
import static org.firstinspires.ftc.teamcode.util.MathFunction.getOptimizedAngle;
import static org.firstinspires.ftc.teamcode.util.Settings.Alliance.BLUE_1;
import static org.firstinspires.ftc.teamcode.util.Settings.Alliance.BLUE_2;

/**
 * to control robot's chassis in another thread
 * @Author WangZiYiDian
 */

public class Chassis implements Runnable {

    public volatile boolean isRunnable;
    public volatile boolean isActive;

    public volatile double MAX_POWER = 0.85;

    private ArrayList<Position> targetPositions = new ArrayList<>();
    private ArrayList<Boolean> needsAccurateAngles = new ArrayList<>();
    private ArrayList<Integer> targetTimes = new ArrayList<>();
    private ArrayList<Integer> waitTimes = new ArrayList<>();
    private ArrayList<Integer> modes = new ArrayList<>();
    private int index = 0;
    public volatile double radius = 8;


    private static final PIDCoefficient positionalAccuratePID = new PIDCoefficient(0.1, 0, 0.5, 0.8, 1);
    private static final PIDCoefficient angularAccuratePID = new PIDCoefficient(0.05, 0, 0.25, 0.1,2);
    private static final PIDCoefficient angularRoughPID = new PIDCoefficient(0.05, 0, 0.25,1,2);
    public volatile PIDCoefficient angularPID = angularAccuratePID.clone();
    public volatile PIDCoefficient positionalPID = positionalAccuratePID.clone();

    private final DcMotorEx motorLF, motorRF, motorLB, motorRB;
    public ThreeWheelOdometry odometry;

    public volatile boolean finishFlag;

    private Alliance alliance;
    public static Position ringStackPosition;
    public static HashMap<String, Position> targetZonePosition;
    public static ArrayList<Position> powerShotEjectPosition;
    public static Position robotStopPosition;
    public static Position robotStartPosition;

    public static final int NOTHING = 0;
    public static final int HIGH_GOAL = 1;
    public static final int POWER_SHOT = 2;
    public static final int WOBBLE_GOAL = 3;
    public static final int TAKE_IN = 4;
    public static final int STOP = 5;
    public static final int PURE_PURSUIT = 6;

    public Chassis(HardwareMap hardwareMap,
                   String motorLfName, String motorRfName, String motorLbName, String motorRbName,
                   String wheelYLeftName, String wheelYRightName, String wheelHorizontalName) {
        motorLF = getMotor(hardwareMap, motorLfName, false);
        motorRF = getMotor(hardwareMap, motorRfName, true);
        motorLB = getMotor(hardwareMap, motorLbName, false);
        motorRB = getMotor(hardwareMap, motorRbName, true);
        odometry = new ThreeWheelOdometry(hardwareMap, wheelYLeftName, wheelYRightName, wheelHorizontalName);
    }

    public Chassis(HardwareMap hardwareMap,
                   String motorLfName, String motorRfName, String motorLbName, String motorRbName) {
        motorLF = getMotor(hardwareMap, motorLfName, false);
        motorRF = getMotor(hardwareMap, motorRfName, true);
        motorLB = getMotor(hardwareMap, motorLbName, false);
        motorRB = getMotor(hardwareMap, motorRbName, true);
    }

    public void start() {
        isActive = true;
    }

    public void stop() {
        isRunnable = false;
        motorMove(0, 0, 0);
    }

    public void pause() {
        isActive = false;
        motorMove(0, 0, 0);
    }

    public void reset() {
        targetPositions = new ArrayList<>();
        targetTimes = new ArrayList<>();
        waitTimes = new ArrayList<>();
        needsAccurateAngles = new ArrayList<>();
        modes = new ArrayList<>();
        index = 0;
    }

    @Override
    public void run() {
        new Thread(odometry).start();
        isRunnable = true;
        while (!isActive && isRunnable) Thread.yield();
        odometry.resetPulseRecord();
        odometry.start();
        while (isRunnable) {
            if (!isActive) {
                Thread.yield();
                continue;
            }
            positionMove();
            finishFlag = true;
            Thread.yield();
        }
        odometry.stop();
    }

    public void addTargetPosition(Position targetPosition, int targetTime, int waitTime, boolean needsAccurateAngle, int mode) {
        targetPositions.add(targetPosition);
        targetTimes.add(targetTime);
        waitTimes.add(waitTime);
        needsAccurateAngles.add(needsAccurateAngle);
        modes.add(mode);
    }

    public void addTargetPosition(Position targetPosition, int mode) {
        int targetTime;
        Position relative = getPreviousPosition().towards(targetPosition);
        targetTime = (int) ((abs(relative.x) + abs(relative.y) + abs(relative.w) / 2)  * 100 / MAX_POWER);
        targetTime = (int) Range.clip(targetTime, 1000, 5000);
        
        switch (mode) {
            case HIGH_GOAL: {
                addTargetPosition(targetPosition, targetTime, 2400, true, HIGH_GOAL);
                break;
            }
            case POWER_SHOT: {
                addTargetPosition(targetPosition, targetTime, 800, true, POWER_SHOT);
                break;
            }
            case WOBBLE_GOAL: {
                addTargetPosition(targetPosition, targetTime, 1500, false, WOBBLE_GOAL);
                break;
            }
            case TAKE_IN: {
                addTargetPosition(targetPosition, targetTime, 0, false, TAKE_IN);
                break;
            }
            case STOP: {
                addTargetPosition(targetPosition, targetTime, 0, false, STOP);
                break;
            }
            case PURE_PURSUIT: {
                addTargetPosition(targetPosition, targetTime, 0, false, PURE_PURSUIT);
                break;
            }
            default: {
                addTargetPosition(targetPosition, targetTime, 0, false, NOTHING);
                break;
            }
        }
    }

    public void addTargetPosition(ArrayList<Position> targetPositions, int mode) {
        for (int i = 0; i < targetPositions.size(); i++)
            addTargetPosition(targetPositions.get(i), mode);
    }

    public void motorMove(double x, double y, double w) {
        double lf = y + x + w;
        double rf = y - x - w;
        double lb = y - x + w;
        double rb = y + x - w;
        double max = max(lf, max(rf, max(lb, rb)));
        if (max > 1) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }
        motorLF.setPower(lf * MAX_POWER);
        motorRF.setPower(rf * MAX_POWER);
        motorLB.setPower(lb * MAX_POWER);
        motorRB.setPower(rb * MAX_POWER);
    }

    public void positionMove() {
        if (index >= targetPositions.size()) return;
        positionalPID = positionalAccuratePID.clone();
        if (needsAccurateAngles.get(index)) angularPID = angularAccuratePID.clone();
        else angularPID = angularRoughPID.clone();
        ElapsedTime runtime = new ElapsedTime();
        finishFlag = false;
        while (true) {
            if (targetPositions.size() == 0 || !isRunnable) {
                motorMove(0, 0, 0);
                return;
            }
            if (!isActive) {
                motorMove(0,0,0);
                Thread.yield();
                continue;
            }

            if (getMode() != PURE_PURSUIT) {
                Position current = getCurrentPosition();
                Position relative = current.towards(targetPositions.get(index));
                motorMove(positionalPID.getValue(relative.length()) * cos(toRadians(relative.angle() - current.w)),
                        positionalPID.getValue() * sin(toRadians(relative.angle() - current.w)),
                        angularPID.getValue(getOptimizedAngle(relative.w)));
                if ((positionalPID.finishFlag && angularPID.finishFlag) || runtime.milliseconds() > targetTimes.get(index)) {
                    finishFlag = true;
                    motorMove(0, 0, 0);
                    if (getMode() == STOP) stop();
                    ElapsedTime temp = new ElapsedTime();
                    while (temp.milliseconds() < waitTimes.get(index)) {
                        if (!isRunnable) return;
                        Thread.yield();
                    }
                    index++;
                    if (index >= targetPositions.size()) return;
                    positionalPID = positionalAccuratePID.clone();
                    if (needsAccurateAngles.get(index)) angularPID = angularAccuratePID.clone();
                    else angularPID = angularRoughPID.clone();
                    runtime.reset();
                    finishFlag = false;
                }
            }
            else {
                finishFlag = false;
                ArrayList<Position> pathPositions = new ArrayList<>();
                pathPositions.add(getCurrentPosition());
                int i = index;
                while (i < modes.size() && modes.get(i) == PURE_PURSUIT) {
                    pathPositions.add(targetPositions.get(i));
                    i++;
                }
                i = 0;
                while (true) {
                    if (!isRunnable) {
                        motorMove(0, 0, 0);
                        return;
                    }
                    if (!isActive) {
                        motorMove(0, 0, 0);
                        Thread.yield();
                        continue;
                    }
                    Position current = getCurrentPosition();
                    Position followPosition;
                    double radius2 = radius;
                    while (getFollowPosition(pathPositions.get(i), pathPositions.get(i + 1), current, radius2) == null)
                        radius2++;
                    while (getFollowPosition(pathPositions.get(i), pathPositions.get(i + 1), current, radius2) != null && radius2 >= radius)
                        radius2--;
                    radius2++;
                    followPosition = getFollowPosition(pathPositions.get(i), pathPositions.get(i + 1), current, radius2);
                    if (current.getDistance(pathPositions.get(i + 1)) < radius2 || runtime.milliseconds() > targetTimes.get(index)) {
                        if (i < pathPositions.size() - 2) {
                            index++;
                            i++;
                        } else followPosition = pathPositions.get(pathPositions.size() - 1);
                        positionalPID.reset();
                        angularPID.reset();
                        runtime.reset();
                    }
                    if (i == pathPositions.size() - 2 && ((positionalPID.finishFlag && angularPID.finishFlag) || runtime.milliseconds() > targetTimes.get(index))) {
                        finishFlag = true;
                        index++;
                        if (index >= targetPositions.size()) return;
                        positionalPID = positionalAccuratePID.clone();
                        if (needsAccurateAngles.get(index)) angularPID = angularAccuratePID.clone();
                        else angularPID = angularRoughPID.clone();
                        runtime.reset();
                        finishFlag = false;
                        break;
                    }
                    Position relative_to_follow = current.towards(Objects.requireNonNull(followPosition));
                    Position relative_to_destination = current.towards(pathPositions.get(i + 1));
                    motorMove(positionalPID.getValue(relative_to_destination.length()) * cos(toRadians(relative_to_follow.angle() - current.w)),
                            positionalPID.getValue() * sin(toRadians(relative_to_follow.angle() - current.w)),
                            angularPID.getValue(getOptimizedAngle(relative_to_destination.w)));
                    Thread.yield();
                }
            }
            Thread.yield();
        }
    }

    public boolean isMotorMoving() {
        return !(abs(motorLB.getPower()) < 0.0001) || !(abs(motorLF.getPower()) < 0.0001) || !(abs(motorRB.getPower()) < 0.0001) || !(abs(motorRF.getPower()) < 0.0001);
    }

    public boolean isStopped() {
        return finishFlag && !isMotorMoving() && isActive;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        odometry.setCurrentPosition(Settings.robotStartPosition.get(alliance));
        ringStackPosition =  Settings.ringStackPosition.get(alliance);
        powerShotEjectPosition = Settings.powerShotEjectPosition.get(alliance);
        targetZonePosition = Settings.targetZonePosition.get(alliance);
        robotStopPosition = Settings.robotStopPosition.get(alliance);
        robotStartPosition = Settings.robotStartPosition.get(alliance);
    }

    public void setCurrentPosition(Position currentPosition) {
        odometry.setCurrentPosition(currentPosition);
    }

    public Position getCurrentPosition() {
        return odometry.getCurrentPosition();
    }

    public int getMode() {
        if (modes.size() == 0 || index >= modes.size()) return -1;
        return modes.get(index);
    }

    private DcMotorEx getMotor(HardwareMap hardwareMap, String motorName, boolean reverseFlag) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (reverseFlag) motor.setDirection(DcMotor.Direction.REVERSE);
        return motor;
    }

    public int getIndex() {
        return index;
    }

    public Position getPreviousPosition() {
        if (targetPositions.size() == 0)
            return getCurrentPosition();
        else return targetPositions.get(targetPositions.size() - 1);
    }

    public double getHighGoalEjectSpeed() {
        double distance;
        Position current = getCurrentPosition();
        if (alliance == BLUE_1 || alliance == BLUE_2)
            distance = current.getDistance(new Position(34.5,140));
        else distance = current.getDistance(new Position(103.5,140));
        return 0.45 * (distance - 100) * (distance - 100) + 2000;
    }
    
    public Position getHighGoalEjectPosition(Position position) {
        if (alliance == BLUE_1 || alliance == BLUE_2)
            return position.setW(position.towards(new Position(34.5,140)).angle() - 90 );
        else return position.setW(position.towards(new Position(103.5,140)).angle() - 90);
    }

    public Position getHighGoalEjectPosition() {
        return getHighGoalEjectPosition(getCurrentPosition());
    }

}
