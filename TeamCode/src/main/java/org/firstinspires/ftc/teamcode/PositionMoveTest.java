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
import static org.firstinspires.ftc.teamcode.util.Chassis.STOP;
import static org.firstinspires.ftc.teamcode.util.Settings.Alliance.BLUE_1;
import static org.firstinspires.ftc.teamcode.util.Settings.maxEjectSpeed;
import static org.firstinspires.ftc.teamcode.util.Settings.pushBackwardIndex;
import static org.firstinspires.ftc.teamcode.util.Settings.pushForwardIndex;
import static org.firstinspires.ftc.teamcode.util.Settings.upIndex;

@Autonomous
@Disabled
public class PositionMoveTest extends OpMode {
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
        chassis.setCurrentPosition(new Position());

        new Thread(chassis).start();

        telemetry.addLine(chassis.getCurrentPosition().toString());
        telemetry.addData("positional", chassis.positionalPID.finishFlag);
        telemetry.addData("angular", chassis.angularPID.finishFlag);
        telemetry.update();

        runtime = new ElapsedTime();

        chassis.addTargetPosition(new Position(23, 69, -90), HIGH_GOAL);
        chassis.addTargetPosition(new Position(0,69,90), STOP);

    }

    @Override
    public void loop() {
        chassis.start();
        telemetry.addLine(chassis.getCurrentPosition().toString());
        telemetry.addData("positional", chassis.positionalPID.finishFlag);
        telemetry.addData("angular", chassis.angularPID.finishFlag);
        telemetry.update();
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

