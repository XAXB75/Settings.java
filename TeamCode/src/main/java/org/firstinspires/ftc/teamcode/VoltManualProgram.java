package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Chassis;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.Settings;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.util.Chassis.HIGH_GOAL;
import static org.firstinspires.ftc.teamcode.util.Chassis.NOTHING;
import static org.firstinspires.ftc.teamcode.util.Chassis.POWER_SHOT;
import static org.firstinspires.ftc.teamcode.util.Chassis.WOBBLE_GOAL;
import static org.firstinspires.ftc.teamcode.util.Chassis.powerShotEjectPosition;
import static org.firstinspires.ftc.teamcode.util.Chassis.robotStopPosition;
import static org.firstinspires.ftc.teamcode.util.Settings.*;
import static org.firstinspires.ftc.teamcode.util.Settings.Alliance.BLUE_1;


@TeleOp(name = "Volt Manual")
@Disabled
public class VoltManualProgram extends LinearOpMode{

    //Parameters
    double x = 0;
    double y = 0;
    double w = 0;

    Chassis chassis;

    DcMotorEx motorEjectI = null;
    DcMotorEx motorEjectII = null;

    DcMotorEx motorTakeIn = null;

    Servo servoPush = null;
    Servo servoUpDown = null;
    Servo servoCatch = null;
    Servo servoDelivery = null;

    DcMotorEx motorLeftVertical = null; //m23
    DcMotorEx motorRightVertical = null; //m21
    DcMotorEx motorHorizontal = null; //m22

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Not ready for start");
        telemetry.update();

        chassis = new Chassis(hardwareMap,
                "m10","m11","m13","m12",
                "m23","m21","m22");

        motorEjectI = hardwareMap.get(DcMotorEx.class,"m20");
        motorEjectII = hardwareMap.get(DcMotorEx.class,"m21");
        motorTakeIn = hardwareMap.get(DcMotorEx.class,"m22");


        motorHorizontal = hardwareMap.get(DcMotorEx.class,"m22");
        motorRightVertical = hardwareMap.get(DcMotorEx.class,"m21");
        motorLeftVertical  = hardwareMap.get(DcMotorEx.class,"m23");

        servoPush = hardwareMap.get(Servo.class,"sP");
        servoUpDown = hardwareMap.get(Servo.class,"sU");

        servoDelivery = hardwareMap.get(Servo.class, "sD");
        servoCatch = hardwareMap.get(Servo.class, "sC");

        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            controllerUpdate();
            driveTrainMove();
            motorOther();
            servoMove();
            printNow();
            idle();
        }
    }

    public void driveTrainMove() {
        double v = 1;
        double deadzone = 0;
        if(gamepad1.right_trigger > 0.2){
            v = 1.2 - gamepad1.right_trigger;
        } else {
            v = 1;
        }
        if(abs(x) > deadzone || abs(y) > deadzone || abs(w) > deadzone){
            chassis.motorMove(x * v,y * v,w * v);
        }else{
            chassis.motorMove(0, 0, 0);
        }
    }

    public void motorOther(){
        if(gamepad1.right_stick_button){
            setEjectMotor(highGoalEjectSpeed);
        }else if(gamepad1.left_stick_button){
            motorEjectI.setPower(0);
            motorEjectII.setPower(0);
        }else if(gamepad1.left_trigger>0.5){
            setEjectMotor(powerShotEjectSpeed);
        }
        //Take In
        if(gamepad1.left_bumper){
            motorTakeIn.setPower(0.9);
        }else if(gamepad1.right_bumper){
            motorTakeIn.setPower(-0.9);
        }else{
            motorTakeIn.setPower(0);
        }
    }

    private void setEjectMotor(double speed){
        motorEjectI.setVelocity(speed);
        motorEjectII.setPower(-speed / maxEjectSpeed);
    }

    public void servoMove(){
        //Push
        if(gamepad1.x){
            servoPush.setPosition(0.9);
        }else{
            servoPush.setPosition(0.5);
        }

        //UpDown
        if(gamepad1.y){
            servoUpDown.setPosition(0.9);
        }

        //gamepad 1 down
        if(gamepad1.right_bumper){
            servoUpDown.setPosition(0.1);
        }

        //wawago
        if(gamepad2.right_bumper){
            servoDelivery.setPosition(0.74);//open 0.69
        } else if (gamepad2.a){
            servoDelivery.setPosition(0.65);
        } else if (gamepad2.right_trigger > 0.1){
            servoDelivery.setPosition(0.62 + 0.1 * gamepad2.right_trigger);
        } else if(gamepad2.left_trigger > 0.1){
            servoDelivery.setPosition(0.2 * gamepad2.left_trigger);
        } else {
            servoDelivery.setPosition(0);//The highest position
        }

        if(gamepad2.left_bumper){
            servoCatch.setPosition(0.65);//open
        } else {
            servoCatch.setPosition(1);//close
        }

    }


    public void controllerUpdate(){

        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        w = -gamepad1.right_stick_x;

        if(gamepad1.dpad_left){
            w = 0.2;
        }else if(gamepad1.dpad_right){
            w = -0.2;
        }else if(gamepad1.dpad_down){
            y = -0.2;
        }else if(gamepad1.dpad_up){
            y = 0.2;
        }

    }

    public void printNow() {
        telemetry.addLine(chassis.getCurrentPosition().toString());
        telemetry.update();
    }

}
