package org.firstinspires.ftc.teamcode.util;

import static java.lang.Math.abs;

/**
 * @Author WangZiYiDian
 */

public enum Color {
    /**
     * the RGB values of some common colors
     */
    RED(255, 0, 0),
    GREEN(0, 255, 0),
    BLUE(0, 0, 255),
    ORANGE(255, 127, 39),
    YELLOW(255, 255, 0),
    CYAN(0, 255, 255),
    MAGENTA(255, 0, 255),
    PURPLE(163, 73, 164),
    BLACK(0, 0, 0),
    WHITE(255, 255, 255),
    GRAY(127, 127, 127);

    private double R, G, B;

    /**
     * constructor of Color
     */
    Color(double R, double G, double B) {
        this.R = R;
        this.G = G;
        this.B = B;
    }

    /**
     * to get the gray scale of this color
     */
    public double toChannel1() {
        return R * 0.299 + G * 0.587 + B * 0.114;
    }

    /**
     * to get the RGB values of this color
     */
    public double[] toChannel3() {
        return new double[] {R, G, B};
    }

    /**
     * to get the 4-channel values of this color
     * commonly used in OpenCv Scalar
     */
    public double[] toChannel4() {
        return new double[] {R, G, B, 0};
    }

    /**
     * to decide whether the input color is the same as this color
     * @param precision between 0 and 1
     */
    public boolean isThisColor(double[] RGB, double precision) {
        double zone = 255 - 255 * precision;
        return abs(RGB[0] - this.R) <= zone && abs(RGB[1] - this.G) <= zone && abs(RGB[2] - this.B) <= zone;
    }

    public boolean isThisColor(double[] RGB) {
        return isThisColor(RGB, 0.85);
    }

    /**
     * for FTC 20/21 season, we need to decide which pixel is part of a ring
     * @param RGB input target color's RGB values
     * @return whether the target color is the color of a ring
     */
    public static boolean isRingColor(double[] RGB) {
        return RGB[0] / RGB[1] > 1 && RGB[0] / RGB[1] < 4 && RGB[1] / RGB[2] > 1.2 && RGB[1] / RGB[2] < 6;
    }
}