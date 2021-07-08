package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.LinkedList;
import java.util.Queue;

public class ServoT implements Runnable{
    private final Servo servo;
    private final Queue<Double> index = new LinkedList<>();
    private final Queue<Long> waitTime = new LinkedList<>();
    private volatile boolean isRunnable;
    public volatile boolean isSleeping = false;

    public ServoT(HardwareMap hardwareMap, String deviceName) {
        servo = hardwareMap.get(Servo.class, deviceName);
    }

    public void setPosition(double index) {
        this.index.offer(index);
    }

    public void sleep(long waitTime) {
        this.waitTime.offer(waitTime);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public void stop() {
        isRunnable = false;
    }

    @Override
    public void run() {
        isRunnable = true;
        while (isRunnable) {
            if (index.size() > 0)
                servo.setPosition(index.poll());
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
