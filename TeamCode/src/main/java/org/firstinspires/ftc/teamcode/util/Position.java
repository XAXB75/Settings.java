package org.firstinspires.ftc.teamcode.util;

import java.util.Objects;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.util.MathFunction.getOptimizedAngle;

public class Position {
    public double x, y, w;  // x, y in Inches; w in Degrees

    public Position() {}
    public Position(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Position(double x, double y, double w) {
        this.x = x;
        this.y = y;
        this.w = w;
    }

    public Position towards(Position targetPosition) {
        return new Position(targetPosition.x - x, targetPosition.y - y, targetPosition.w - w);
    }
    public Position add(Position position) {
        return new Position(position.x + x, position.y + y, position.w + w);
    }
    public Position add(double x, double y, double w) {
        return add(new Position(x, y, w));
    }
    public Position setW(double w) {
        return new Position(x, y, w);
    }
    public Position setX(double x) {
        return new Position(x, y, w);
    }
    public Position setY(double y) {
        return new Position(x, y, w);
    }

    public double length() {
        return Math.sqrt(x * x + y * y);
    }
    public double angle() {
        return Math.toDegrees(Math.atan2(y, x));
    }


    public Position getExtendedPosition(double x, double y) {
        Position position = add(x * cos(toRadians(w)) - y * sin(toRadians(w)), x * sin(toRadians(w)) + y * cos(toRadians(w)), 0);
        return position.setW(towards(position).angle() - 90);
    }

    public double getDistance(Position position) {
        return towards(position).length();
    }


    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Position position = (Position) obj;
        return Double.compare(position.x, x) == 0 &&
                Double.compare(position.y, y) == 0 &&
                Double.compare(position.w, w) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, w);
    }

    @Override
    public String toString() {
        w = getOptimizedAngle(w);
        return "Position (" + x + ", " + y + ", " + w + ')';
    }

}
