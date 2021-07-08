package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Point;

import java.util.HashSet;

public class ConnectedComponent {
    private final HashSet<Point> points;
    private int left, right, top, bottom;

    public ConnectedComponent() {
        points = new HashSet<>();
        left = right = top = bottom = -5;
    }

    public int size() {
        return points.size();
    }

    public boolean isEmpty() {
        return points.isEmpty();
    }

    public void add(Point p) {
        points.add(p);
        left = (left < 0) ? (int)p.x : Math.min(left, (int)p.x);
        right = (right < 0) ? (int)p.x : Math.max(right, (int)p.x);
        top = (top < 0) ? (int)p.y : Math.min(top, (int)p.y);
        bottom = (bottom < 0) ? (int)p.y : Math.max(bottom, (int)p.y);
    }

    public int left() {
        return left;
    }
    public int right() {
        return right;
    }
    public int top() {
        return top;
    }
    public int bottom() {
        return bottom;
    }
    public Point downMost() {
        double x = 0;
        int t = 0;
        for (Point p : points)
            if (p.y >= bottom - 4) {
                x += p.x;
                t++;
            }
        x /= t;
        return new Point(x, bottom);
    }

    public boolean contains(Point p) {
        return points.contains(p);
    }
}