package org.firstinspires.ftc.teamcode.util;

import static java.lang.Math.abs;
import static java.lang.Math.floor;
import static java.lang.Math.sqrt;

/**
 * all the math functions
 * @Author Candela
 */
public class MathFunction {
    public static double getOptimizedAngle(double angle) {
        return angle - floor((angle + 180) / 360) * 360;
    }

    public static Position getFollowPosition(Position startPosition, Position endPosition, Position current, double radius) {
        if (abs(startPosition.x - endPosition.x) < 0.02) startPosition.x = endPosition.x + 0.02;
        double k = (startPosition.y - endPosition.y) / (startPosition.x - endPosition.x);

        double x = startPosition.x - current.x;
        double y = startPosition.y - current.y;

        double A = 1 + k * k;
        double B = 2 * k * y - 2 * k * k * x;
        double C = k * k * x * x - 2 * y * k * x + y * y - radius * radius;

        if (B * B < 4 * A * C) return null;
        int direction = (endPosition.x > startPosition.x) ? 1 : -1;
        double xRoot = (-B + direction * sqrt(B * B - 4 * A * C ))/(2 * A);
        double yRoot = k * (xRoot - x) + y;
        xRoot += current.x;
        yRoot += current.y;

        return new Position(xRoot, yRoot);
    }
}
