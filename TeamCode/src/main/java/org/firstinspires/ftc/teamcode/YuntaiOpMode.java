package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Chassis;
import org.firstinspires.ftc.teamcode.util.MathFunction;
import org.firstinspires.ftc.teamcode.util.MotorT;
import org.firstinspires.ftc.teamcode.util.MyQueue;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.ThreeWheelOdometry;

import static java.lang.Math.abs;
import static java.lang.Math.max;

@TeleOp(group = "OpMode", name = "云台")
@Config
public class YuntaiOpMode extends OpMode {

    ElapsedTime previousX_time = new ElapsedTime();

    Chassis chassis;
    ThreeWheelOdometry odometry;
    DcMotorEx motorLaunchR, motorLaunchL, motorWheel;
    MotorT motorTimingBelt;
    Servo servoPush, servoOrientationR, servoOrientationL, servoCatch, servoDelivery, servoUpDown;
    ColorSensor colorSensor;
    double x, y, w;

    public static double forwardIndex = 0.25;
    public static double backIndex = 0.6;
    public static double downIndex = 1;
    public static double handUpIndex = 0.2;
    public static double handDownIndex = 0.8;
    public static double openIndex = 0.1;
    public static double closeIndex = 0.72;
    public static double[] upIndex = new double[]{0,0,0.02,0.05};
    public static double yuntaiSpeed = 2.5;

    double launchSpeed = 2200;
    double launchSpeed_Delta = 0;

    double launchAngle_Delta = 0;
    double launchAngle = 90;

    int ringNum = 0;
    boolean isLifting = false;
    int isLaunching = 0;
    boolean isTaking = false;

    MyQueue color = new MyQueue(5);
    @Override
    public void init() {
        chassis = new Chassis(hardwareMap, "LF", "RF", "LB", "RB");
        odometry = new ThreeWheelOdometry(hardwareMap, "LF", "RB","LB");

        motorLaunchL = hardwareMap.get(DcMotorEx.class, "LaunchL");
        motorLaunchR = hardwareMap.get(DcMotorEx.class, "LaunchR");
        motorWheel = hardwareMap.get(DcMotorEx.class, "Wheel");
        motorTimingBelt = new MotorT(hardwareMap, "Timing");

        servoOrientationR = hardwareMap.get(Servo.class, "OR");
        servoOrientationL = hardwareMap.get(Servo.class, "OL");

        servoPush = hardwareMap.get(Servo.class, "P");
        servoUpDown = hardwareMap.get(Servo.class, "U");
        servoCatch = hardwareMap.get(Servo.class, "C");
        servoDelivery = hardwareMap.get(Servo.class, "D");

        colorSensor = hardwareMap.get(ColorSensor.class, "Color");

        motorTimingBelt.setDirection(DcMotor.Direction.REVERSE);
        motorWheel.setDirection(DcMotor.Direction.REVERSE);
        motorLaunchL.setDirection(DcMotor.Direction.REVERSE);

        motorLaunchL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLaunchR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odometry.setCurrentPosition(new Position(9,9));
        new Thread(odometry).start();

        new Thread(motorTimingBelt).start();

        servoUpDown.setPosition(downIndex);
        servoPush.setPosition(backIndex);
        servoOrientationR.setPosition(launchAngle /180.0);
        servoOrientationL.setPosition(launchAngle /180.0);
        servoDelivery.setPosition(handUpIndex);
        servoCatch.setPosition(closeIndex);//close

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        updateGamePadData();
        driveTrainMove();

        servoMove();
        launchPart();
        takeInPart();

        ringNumUpdate();
//        positioning();

        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("RingNum", ringNum);
        telemetry.update();
    }

    @Override
    public void stop() {
        motorTimingBelt.stop();
    }

    public void updateGamePadData(){
        x = -gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        w = gamepad1.right_stick_x;
    }

    public void driveTrainMove(){
        double v;
        double deadzone = 0;
        if(gamepad1.left_trigger > 0.2){
            v = 1.2 - gamepad1.left_trigger;
        } else {
            v = 1;
        }
        if(abs(x) > deadzone || abs(y) > deadzone || abs(w) > deadzone){
            chassis.motorMove(x * v,y * v,w * v);
        }else{
            chassis.motorMove(0, 0, 0);
        }
    }

    public void servoMove() {
        if (gamepad1.dpad_left) {
            launchAngle_Delta -= 0.2;
        } else if (gamepad1.dpad_right) {
            launchAngle_Delta += 0.2;
        }

        launchAngle_Delta += (gamepad2.right_trigger - gamepad2.left_trigger) / 2;

        launchAngle = Range.clip(MathFunction.getOptimizedAngle(180
                - odometry.getCurrentPosition().getExtendedPosition(0,-2).
                towards(new Position(34.5,140)).angle()
                + odometry.getCurrentPosition().w + launchAngle_Delta), 40, 140);
        launchAngle = servoOrientationL.getPosition() * 180 + (launchAngle > servoOrientationL.getPosition() * 180 ? yuntaiSpeed : -yuntaiSpeed);

        launchAngle = 90 + launchAngle_Delta;

        servoOrientationR.setPosition(launchAngle /180.0);
        servoOrientationL.setPosition(launchAngle /180.0);

        if (!gamepad1.atRest() || gamepad1.x || gamepad1.left_stick_button || gamepad1.left_bumper || gamepad1.right_bumper)
            isLifting = false;
//        if(gamepad1.x || gamepad2.x){
//            servoPush.setPosition(forwardIndex);
//            if (previousX_time.milliseconds() > 450 && servoUpDown.getPosition() < 0.6) {
//                    ringNum = max(ringNum - 1, 0);
//                    servoUpDown.sleep(150);
//                    servoUpDown.setPosition(0.6);
//                    servoUpDown.sleep(300);
//                    servoUpDown.setPosition(upIndex[ringNum]);
//            }
//            previousX_time.reset();
//        }else {
//            if (servoUpDown.getPosition() == downIndex)
//                servoPush.setPosition(backIndex);
//            else if (previousX_time.milliseconds() > 350) {
//                servoPush.setPosition(backIndex);
//            }
//        }
        if (gamepad1.x || gamepad2.x) {
            servoPush.setPosition(forwardIndex);
            ringNum = max(ringNum - 1, 0);
            servoUpDown.setPosition(upIndex[ringNum]);
        } else
            servoPush.setPosition(backIndex);

        if((gamepad1.y || gamepad2.dpad_up) && !isTaking) {
            if (servoUpDown.getPosition() == downIndex) isLifting = true;
            servoUpDown.setPosition(upIndex[ringNum]);
        }
        if (gamepad1.dpad_down || gamepad2.dpad_down){
            servoUpDown.setPosition(downIndex);
        }

        if (gamepad2.right_bumper) {
            servoDelivery.setPosition(handDownIndex);
        } else {
            servoDelivery.setPosition(handUpIndex);
        }

        if(gamepad2.left_bumper){
            servoCatch.setPosition(openIndex);//open
        } else {
            servoCatch.setPosition(closeIndex);//close
        }
    }

    public void launchPart(){
        if(gamepad1.right_stick_button){
            isLaunching = 1;
        } else if(gamepad1.left_stick_button){
            isLaunching = 0;
        }

//        launchSpeed_Delta += 2 * gamepad2.right_stick_y;

//        launchSpeed = 2300 - 7 * (odometry.getCurrentPosition().getDistance(new Position(34.5,140)) - 50) + launchSpeed_Delta;

        if (gamepad2.y)
            launchSpeed = 2200;
        else if (gamepad2.b)
            launchSpeed = 2000;
        else if (gamepad2.a)
            launchSpeed = 1800;

        if (isLaunching > 0) {
            motorLaunchL.setVelocity(launchSpeed);
            motorLaunchR.setVelocity(launchSpeed);
        } else {
            motorLaunchL.setPower(0);
            motorLaunchR.setPower(0);
        }
    }
    public void takeInPart(){
        if (isLifting) {
            motorTimingBelt.setPower(0.3);
            motorWheel.setPower(0.3);
        } else if (gamepad1.left_bumper) {
            motorTimingBelt.setPower(-1);
            motorWheel.setPower(-1);
        } else if (isTaking) {
            if (ringNum < 3) {
                motorTimingBelt.setPower(0.9);
                motorWheel.setPower(0.9);
                if (!motorTimingBelt.isSleeping) motorTimingBelt.sleep(500);
            } else {
                motorTimingBelt.setPower(-1);
                motorWheel.setPower(-1);
            }
        } else if (gamepad1.right_bumper) {
            if (ringNum < 3) {
                servoUpDown.setPosition(downIndex);
                motorTimingBelt.setPower(0.9);
                motorWheel.setPower(0.9);
            } else {
                motorTimingBelt.setPower(-1);
                motorWheel.setPower(-1);
            }
        } else {
            motorTimingBelt.setPower(0);
            motorWheel.setPower(0);
        }
    }

    public void ringNumUpdate() {
        boolean tmp = isTaking;
        isTaking = colorSensor.green() > colorSensor.blue() * 2;
        if (tmp && !isTaking) ringNum = Math.min(ringNum + 1, 3);
    }

    public void positioning() {
        if (gamepad1.a) {
            launchAngle_Delta = 0;
            launchSpeed_Delta = 0;
            double currentX = odometry.getCurrentPosition().x;
            double currentY = odometry.getCurrentPosition().y;
            if (currentX < 14) odometry.setCurrentPosition(new Position(9, currentY));
            if (currentX > 124) odometry.setCurrentPosition(new Position(129, currentY));
            if (currentY < 12) odometry.setCurrentPosition(new Position(currentX, 9));
            if (currentX < 122) odometry.setCurrentPosition(new Position(currentX, 129));
        }
    }

}
