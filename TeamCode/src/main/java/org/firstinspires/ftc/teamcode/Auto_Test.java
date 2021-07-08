package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Chassis;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.Settings.Alliance;

import static org.firstinspires.ftc.teamcode.util.Chassis.HIGH_GOAL;
import static org.firstinspires.ftc.teamcode.util.Chassis.POWER_SHOT;
import static org.firstinspires.ftc.teamcode.util.Chassis.STOP;
import static org.firstinspires.ftc.teamcode.util.Chassis.WOBBLE_GOAL;
import static org.firstinspires.ftc.teamcode.util.Chassis.powerShotEjectPosition;
import static org.firstinspires.ftc.teamcode.util.Chassis.robotStartPosition;
import static org.firstinspires.ftc.teamcode.util.Chassis.robotStopPosition;
import static org.firstinspires.ftc.teamcode.util.Chassis.targetZonePosition;
import static org.firstinspires.ftc.teamcode.util.Settings.Alliance.BLUE_1;
import static org.firstinspires.ftc.teamcode.util.Settings.deliveryDownIndex;
import static org.firstinspires.ftc.teamcode.util.Settings.deliveryUpIndex;
import static org.firstinspires.ftc.teamcode.util.Settings.downIndex;
import static org.firstinspires.ftc.teamcode.util.Settings.maxEjectSpeed;
import static org.firstinspires.ftc.teamcode.util.Settings.pickCloseIndex;
import static org.firstinspires.ftc.teamcode.util.Settings.pickOpenIndex;
import static org.firstinspires.ftc.teamcode.util.Settings.powerShotEjectSpeed;
import static org.firstinspires.ftc.teamcode.util.Settings.pushBackwardIndex;
import static org.firstinspires.ftc.teamcode.util.Settings.pushForwardIndex;
import static org.firstinspires.ftc.teamcode.util.Settings.upIndex;

@Autonomous
@Disabled
public class Auto_Test extends OpMode {
    Chassis chassis;

    DcMotorEx motorEjectI;
    DcMotorEx motorEjectII;
    DcMotorEx motorTakeIn;

    Servo servoPush;
    Servo servoUpDown;
    Servo servoDelivery;
    Servo servoCatch;

    Alliance alliance = BLUE_1;

    ElapsedTime runtime;

    int index = -1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        chassis = new Chassis(hardwareMap,
                "m10","m11","m13","m12",
                "m23","m21","m22");
        motorEjectI = hardwareMap.get(DcMotorEx.class,"m20");
        motorEjectII = hardwareMap.get(DcMotorEx.class, "m21");
        motorTakeIn = hardwareMap.get(DcMotorEx.class, "m22");

        servoPush = hardwareMap.get(Servo.class,"sP");
        servoUpDown = hardwareMap.get(Servo.class,"sU");
        servoDelivery = hardwareMap.get(Servo.class, "sD");
        servoCatch = hardwareMap.get(Servo.class, "sC");

        chassis.stop();
        chassis.reset();
        chassis.setAlliance(BLUE_1);
        chassis.setCurrentPosition(robotStartPosition);

        new Thread(chassis).start();

        runtime = new ElapsedTime();

        chassis.addTargetPosition(powerShotEjectPosition, POWER_SHOT);
        chassis.addTargetPosition(targetZonePosition.get("None"), WOBBLE_GOAL);
        chassis.addTargetPosition(robotStopPosition, STOP);


        servoUpDown.setPosition(upIndex);
        servoDelivery.setPosition(deliveryUpIndex);
        servoCatch.setPosition(pickCloseIndex);
        servoPush.setPosition(pushBackwardIndex);
    }

    @Override
    public void loop() {
        chassis.start();
        if (!chassis.isRunnable) {
            servoUpDown.setPosition(downIndex);
            servoDelivery.setPosition(deliveryUpIndex);
            servoCatch.setPosition(pickCloseIndex);
            servoPush.setPosition(pushBackwardIndex);
            motorTakeIn.setPower(0);
            setEjectMotor(0);
        }
        else
            setEjectMotor(powerShotEjectSpeed);
        if (chassis.finishFlag && index < chassis.getIndex()) {
            index = chassis.getIndex();
            switch (chassis.getMode()) {
                case POWER_SHOT: {
                    singlePush();
                    break;
                }
                case WOBBLE_GOAL: {
                    servoUpDown.setPosition(downIndex);
                    servoDelivery.setPosition(deliveryDownIndex);
                    sleep(600);
                    servoCatch.setPosition(pickOpenIndex);
                    sleep(400);
                    servoDelivery.setPosition(deliveryUpIndex);
                    sleep(800);
                    servoCatch.setPosition(pickCloseIndex);
                    break;
                }
            }
        }
        telemetry.addLine(chassis.getCurrentPosition().toString());
    }



    @Override
    public void stop() {
        chassis.stop();
    }

    private void singlePush(){
        sleep(100);
        servoPush.setPosition(pushForwardIndex);
        sleep(500);
        servoPush.setPosition(pushBackwardIndex);
    }

    private void setEjectMotor(double speed){
        motorEjectI.setVelocity(speed);
        motorEjectII.setPower(-speed / maxEjectSpeed);
    }

    private void loadRing() {
        motorTakeIn.setPower(0);
        servoUpDown.setPosition(upIndex);
        sleep(200);
    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}

