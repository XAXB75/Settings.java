package org.firstinspires.ftc.teamcode.util;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

public class PIDCoefficient {
    private final double P, I, D;
    private int times;
    private double totalError, previousError;
    private double value;
    private final double finishRange;
    public volatile boolean finishFlag;
    private final double out_min, out_max;
    private final double integral_max;


    public PIDCoefficient(double P, double I, double D, double finishRange, double out_min, double out_max, double integral_max) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.finishRange = finishRange;
        this.out_min = out_min;
        this.out_max = out_max;
        this.integral_max = integral_max;
        finishFlag = false;
    }
    public PIDCoefficient(double P, double I, double D, double finishRange, double out_max) {
        this(P, I, D, finishRange, 0, out_max, Double.POSITIVE_INFINITY);
    }
    public PIDCoefficient(double P, double I, double D, double finishRange) {
        this(P, I, D, finishRange, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public void reset() {
        totalError = previousError = 0;
        times = 0;
        value = 0;
        finishFlag = false;
    }

    public void update(double error) {
        finishFlag = abs(error) < finishRange;
        totalError += min(error, integral_max);
        times++;
        if (previousError == 0) previousError = error;
        value = P * error + I * totalError / times + D * (error - previousError);
        previousError = error;
    }

    public double getValue() {
        value = max(-out_max, min(out_max, value));
        if (abs(value) < out_min) value *= out_min / abs(value);
        return value;
    }
    public double getValue(double error) {
        update(error);
        return getValue();
    }

    @Override
    public String toString() {
        return "PID " + getValue();
    }

    public PIDCoefficient clone() {
        return new PIDCoefficient(P,I,D,finishRange,out_min,out_max,integral_max);
    }
}
