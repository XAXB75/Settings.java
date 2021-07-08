package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.LinkedList;
import java.util.Queue;

public class MotorT implements Runnable{
    private final DcMotorEx motor;
    private final Queue<Double> index = new LinkedList<>();
    private final Queue<Long> waitTime = new LinkedList<>();
    private volatile boolean isRunnable;
    public volatile boolean isSleeping = false;

    public MotorT(HardwareMap hardwareMap, String deviceName) {
        motor = hardwareMap.get(DcMotorEx.class, deviceName);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public void setPower(double index) {
        this.index.offer(index);
    }

    public void sleep(long waitTime) {
        this.waitTime.offer(waitTime);
    }

    public double getPower() {
        return motor.getPower();
    }

    public void stop() {
        isRunnable = false;
    }

    @Override
    public void run() {
        isRunnable = true;
        while (isRunnable) {
            if (index.size() > 0)
                motor.setPower(index.poll());
            if (waitTime.size() > 0) {
                isSleeping = true;
                try {
                    Thread.sleep(waitTime.poll());
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                isSleeping = false;
            }
            Thread.yield();
        }
    }
}
