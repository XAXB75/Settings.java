package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Objects;

public class Ring {
    private final Position position;
    private final Point downMost;
    private final String ringCase;

    private final int left, right, top, bottom;

    public Ring(int left, int right, int up, int down, Point downMost) {
        this.left = left;
        this.right = right;
        this.top = up;
        this.bottom = down;
        this.downMost = downMost;
        double x = downMost.x;
        double y = downMost.y;
        this.position = new Position((-0.105027791 * x -0.011669755 * y + 18.81747929)/(0.000413387 * x -0.013068208 * y + 1),
                (-0.054849212 * x -0.061229618 * y -10.49356972)/(-0.000764579 * x -0.009489632
                        * y + 1));
        if (1.0 * (right - left) / (down - up) > 1.5) this.ringCase = "Single";
        else this.ringCase = "Quad";
    }

    public void drawFrame(Mat mat) {
        Imgproc.rectangle(mat,
                new Point(left - 1, top - 1),
                new Point(right + 1, bottom + 1),
                new Scalar(Color.GREEN.toChannel4()),
                2,
                8,
                0 );
        Imgproc.rectangle(mat,
                new Point(downMost.x - 1, downMost.y - 1),
                new Point(downMost.x + 1, downMost.y + 1),
                new Scalar(Color.GREEN.toChannel4()),
                1,
                8,
                0 );
        Imgproc.putText(mat,
                getRingCase(),
                new Point(left - 3, bottom + 15),
                Imgproc.FONT_HERSHEY_COMPLEX,
                0.5,
                new Scalar(Color.GREEN.toChannel4()));
    }

    public double getDistance() {
        return new Position().getDistance(position);
    }
    public Position getRelativePosition() {
        return position;
    }
    public int getWidth() {
        return right - left;
    }
    public int getHeight() {
        return bottom - top;
    }
    public double getRatio() {
        return 1.0 * getWidth() / getHeight();
    }
    public String getRingCase() {
        return ringCase;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Ring ring = (Ring) obj;
        return left == ring.left &&
                right == ring.right &&
                top == ring.top &&
                bottom == ring.bottom;
    }

    @Override
    public int hashCode() {
        return Objects.hash(left, right, top, bottom);
    }

    @Override
    public String toString() {
        return "Ring " + ringCase + " at (" + position.x + ", " + position.y + ")";
    }
}