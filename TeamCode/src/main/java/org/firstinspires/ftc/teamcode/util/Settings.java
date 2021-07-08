package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.util.Settings.Alliance.BLUE_1;
import static org.firstinspires.ftc.teamcode.util.Settings.Alliance.BLUE_2;
import static org.firstinspires.ftc.teamcode.util.Settings.Alliance.RED_1;
import static org.firstinspires.ftc.teamcode.util.Settings.Alliance.RED_2;

public class Settings {
    public enum Alliance {
        BLUE_1,
        BLUE_2,
        RED_1,
        RED_2
    }
    public static final HashMap<Alliance, Position> ringStackPosition = new HashMap<Alliance, Position>() {
        {
            put(BLUE_1, new Position(34.5, 47));
            put(BLUE_2, new Position(34.5, 47));
            put(RED_1, new Position(103.5, 47));
            put(RED_2, new Position(103.5, 47));
        }
    };
    public static final HashMap<Alliance, Position> robotStartPosition = new HashMap<Alliance, Position>() {
        {
            put(BLUE_1, new Position(45, 7));
            put(BLUE_2, new Position(24, 7));
            put(RED_1, new Position(93, 7));
            put(RED_2, new Position(114, 7));
        }
    };
    public static final HashMap<Alliance, HashMap<String, Position>> targetZonePosition = new HashMap<Alliance, HashMap<String, Position>>() {
        {
            put(BLUE_1, new HashMap<String, Position>() {
                {
                    put("None", new Position(23 + 7, 4 * 23 - 8, -90));
                    put("Single", new Position(2 * 23 + 7, 5 * 23 - 8, -90));
                    put("Quad", new Position(23 + 7, 6 * 23 - 8, -90));
                }
            });
            put(BLUE_2, new HashMap<String, Position>() {
                {
                    put("None", new Position(23 - 3, 3 * 23 - 7 , 180));
                    put("Single", new Position(23 - 7 ,  4 * 23 + 8, 90));
                    put("Quad", new Position(23 - 3, 5 * 23 - 7, 180));
                }
            });
            put(RED_1, new HashMap<String, Position>() {
                {
                    put("None", new Position(5 * 23 - 7, 4 * 23 - 8, 90));
                    put("Single", new Position(4 * 23 - 7, 5 * 23 - 8, 90));
                    put("Quad", new Position(5 * 23 - 7, 6 * 23 - 8, 90));
                }
            });
            put(RED_2, new HashMap<String, Position>() {
                {
                    put("None", new Position(5 * 23 + 11, 3 * 23 - 7, 180));
                    put("Single", new Position(5 * 23 + 7, 4 * 23 + 8, -90));
                    put("Quad", new Position(5 * 23 + 11, 5 * 23 - 7, 180));
                }
            });
        }
    };

    public static final HashMap<Alliance, ArrayList<Position>> powerShotEjectPosition = new HashMap<Alliance, ArrayList<Position>>() {
        {
            double firstAngle = -5.8;
            double secondAngle = 0;
            double thirdAngle = 5.8;
            Position position = new Position(57.3, 69);
            Position finalPosition = position;
            double finalFirstAngle = firstAngle;
            double finalSecondAngle = secondAngle;
            double finalThirdAngle = thirdAngle;
            put(BLUE_1, new ArrayList<Position>() {
                {
                    add(finalPosition.setW(finalFirstAngle));
                    add(finalPosition.setW(finalSecondAngle));
                    add(finalPosition.setW(finalThirdAngle));
                }
            });
            put(BLUE_2, new ArrayList<Position>() {
                {
                    add(finalPosition.setW(finalFirstAngle));
                    add(finalPosition.setW(finalSecondAngle));
                    add(finalPosition.setW(finalThirdAngle));
                }
            });
            firstAngle = 5.8;
            secondAngle = 0;
            thirdAngle = -5.8;
            position = new Position(80.5, 69);
            Position finalPosition1 = position;
            double finalFirstAngle1 = firstAngle;
            double finalSecondAngle1 = secondAngle;
            double finalThirdAngle1 = thirdAngle;
            put(RED_1, new ArrayList<Position>() {
                {
                    add(finalPosition1.setW(finalFirstAngle1));
                    add(finalPosition1.setW(finalSecondAngle1));
                    add(finalPosition1.setW(finalThirdAngle1));
                }
            });
            put(RED_2, new ArrayList<Position>() {
                {
                    add(finalPosition1.setW(finalFirstAngle1));
                    add(finalPosition1.setW(finalSecondAngle1));
                    add(finalPosition1.setW(finalThirdAngle1));
                }
            });
        }
    };

    public static final HashMap<Alliance, Position> robotStopPosition =  new HashMap<Alliance, Position>() {
        {
            put(BLUE_1, new Position(2 * 23 + 11.5, 80.5, 0));
            put(BLUE_2, new Position(23 + 11.5, 80.5, 0));
            put(RED_1, new Position(4 * 23 - 11.5, 80.5, 0));
            put(RED_2, new Position(5 * 23 - 11.5, 80.5, 0));
        }
    };

    public static final double powerShotEjectSpeed = 2050;
    public static final double highGoalEjectSpeed = 2400;
    public static final double maxEjectSpeed = 2450;

    public static final double pushForwardIndex = .9;
    public static final double pushBackwardIndex = .5;
    public static final double upIndex = .9;
    public static final double downIndex = .1;
    public static final double pickCloseIndex = 1;
    public static final double pickHalfCloseIndex = .6;
    public static final double pickOpenIndex = 0.65;
    public static final double deliveryDownIndex = .69;
    public static final double deliveryUpIndex = 0;
}
