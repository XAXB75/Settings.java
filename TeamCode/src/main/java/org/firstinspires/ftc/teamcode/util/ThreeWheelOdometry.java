
package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

public class ThreeWheelOdometry implements Runnable{

    private static final double wheelDiameter = 2;     //in inch
    private static final double ppr = 4096;
    private static final double inchPerPulse = ((wheelDiameter * Math.PI) / ppr);
    private static final double wheelDistance = 14.41;

    private double yRightPulse, yLeftPulse, xPulse = 0;
    private double previousYRightPulse, previousYLeftPulse, previousXPulse;

    private boolean isRunnable;
    private boolean isActive;

    private Position currentPosition;
    private final DcMotor wheelYLeft, wheelYRight, wheelX;

    /**
     * Constructor for ThreeWheelOdometry
     * @param wheelYLeftName left vertical dead wheel
     * @param wheelYRightName right vertical dead wheel
     * @param wheelXName horizontal dead wheel
     */
    public ThreeWheelOdometry(HardwareMap hardwareMap, String wheelYLeftName, String wheelYRightName, String wheelXName){
        wheelYLeft = hardwareMap.get(DcMotor.class, wheelYLeftName);
        wheelYRight = hardwareMap.get(DcMotor.class, wheelYRightName);
        wheelX = hardwareMap.get(DcMotor.class, wheelXName);
        wheelYRight.setDirection(DcMotor.Direction.REVERSE);
        wheelYLeft.setDirection(DcMotor.Direction.REVERSE);
        updatePulse();
        resetPulseRecord();
    }
    //PULSE UPDATE METHODS

    /**
     * get the current pulse of the horizontal dead wheel
     * @return the current pulse of the horizontal dead wheel
     */
    public int getXPulse(){
        return wheelX.getCurrentPosition();
    }

    /**
     * get the current pulse of the left vertical dead wheel
     * @return  the current pulse of the left vertical dead wheel
     */
    public int getYLeftPulse(){
        return wheelYLeft.getCurrentPosition();
    }

    /**
     * get the current pulse of the right vertical dead wheel
     * @return  the current pulse of the right vertical dead wheel
     */
    public int getYRightPulse(){
        return wheelYRight.getCurrentPosition();
    }

    /**
     * record all the pulse in global access variables
     */
    private void updatePulse(){
        xPulse = getXPulse();
        yLeftPulse = getYLeftPulse();
        yRightPulse = getYRightPulse();
    }

    public void resetPulseRecord(){
        previousXPulse = xPulse;
        previousYLeftPulse = yLeftPulse;
        previousYRightPulse = yRightPulse;
    }

    //DISPLACEMENT METHODS
    /**
     * get the robot x displacement increment
     * @return x increment, in inch
     */

    private double getXChange(){
        return (xPulse - previousXPulse) * inchPerPulse;
    }

    /**
     * get the robot y displacement increment
     * @return y increment, in inch
     */
    private double getYChange(){
        return .5 * (
                (yRightPulse - previousYRightPulse) + (yLeftPulse - previousYLeftPulse)
        ) * inchPerPulse;
    }

    /**
     * return the angle based on current saved pulse record
     * @return the current angle of the robot, in degree
     */
    public double getAngleChange(){
        return ((yRightPulse - previousYRightPulse) - (yLeftPulse - previousYLeftPulse))
                * inchPerPulse / wheelDistance / Math.PI * 180;
    }

    /**
     * reset the robot position manually
     */
    public void setCurrentPosition(Position currentPosition){
        this.currentPosition = currentPosition;
    }

    /**
     * update the position of the robot
     */
    public void updatePosition(){
        updatePulse();
        double xChange = getXChange();
        double yChange = getYChange();
        double angleChange = getAngleChange();

        double sin = sin(toRadians(currentPosition.w + angleChange / 2));
        double cos = cos(toRadians(currentPosition.w + angleChange / 2));

        currentPosition = currentPosition.add(cos * xChange - sin * yChange, cos * yChange + sin * xChange, angleChange);

        resetPulseRecord();
    }


    public Position getCurrentPosition() {
        return currentPosition;
    }

    //MultiThreading Methods

    /**
     * restart the thread is stopped
     */
    public void start() {
        isActive = true;
    }
    public void stop() {
        isRunnable = false;
    }
    public void pause() {
        isActive = false;
    }

    @Override
    public void run() {
        isRunnable = true;
        while (isRunnable) {
            if (!isActive || currentPosition == null) {
                Thread.yield();
                continue;
            }
            updatePosition();
            Thread.yield();
        }
    }
}