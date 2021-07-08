package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Chassis;
import org.firstinspires.ftc.teamcode.util.FindRingPipeline;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.Settings.Alliance;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.util.Chassis.HIGH_GOAL;
import static org.firstinspires.ftc.teamcode.util.Chassis.NOTHING;
import static org.firstinspires.ftc.teamcode.util.Chassis.STOP;
import static org.firstinspires.ftc.teamcode.util.Chassis.TAKE_IN;
import static org.firstinspires.ftc.teamcode.util.Chassis.WOBBLE_GOAL;
import static org.firstinspires.ftc.teamcode.util.Chassis.ringStackPosition;
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
import static org.firstinspires.ftc.teamcode.util.Settings.pushBackwardIndex;
import static org.firstinspires.ftc.teamcode.util.Settings.pushForwardIndex;
import static org.firstinspires.ftc.teamcode.util.Settings.upIndex;

@Autonomous
@Disabled
public class VoltAuto_BLUE_1 extends LinearOpMode {
    Chassis chassis;

    OpenCvCamera webcam;

    DcMotorEx motorEjectI;
    DcMotorEx motorEjectII;
    DcMotorEx motorTakeIn;

    Servo servoPush;
    Servo servoUpDown;
    Servo servoDelivery;
    Servo servoCatch;

    Alliance alliance = BLUE_1;
    String ringCase = "None";

    ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initMotorServo();

        chassis.stop();
        chassis.reset();
        chassis.setAlliance(alliance);
        chassis.setCurrentPosition(robotStartPosition);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        FindRingPipeline pipeline = new FindRingPipeline();
        pipeline.setFrame(0,200,90,240);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));

        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        new Thread(chassis).start();

        while (pipeline.getRings() == null);

        servoUpDown.setPosition(upIndex);
        servoDelivery.setPosition(deliveryUpIndex);
        servoCatch.setPosition(pickCloseIndex);
        servoPush.setPosition(pushBackwardIndex);


        waitForStart();

        runtime = new ElapsedTime();
        while (runtime.milliseconds() < 1000) {
            if (pipeline.getRings() != null) {
                if (!pipeline.getRings().isEmpty()) ringCase = pipeline.getRings().peek().getRingCase();
                break;
            }
        }

        Position position = robotStartPosition.add(15,46,0);
        position = chassis.getHighGoalEjectPosition(position);

        chassis.addTargetPosition(position, HIGH_GOAL);
        if (ringCase != "None") {
            double angle = position.towards(ringStackPosition).angle();
            Position ringTakeInStartPosition = new Position(ringStackPosition.x - 10 * cos(toRadians(angle)), ringStackPosition.y - 10 * sin(toRadians(angle)), angle - 90);
            Position ringTakeInEndPosition = new Position(ringStackPosition.x - 3 * cos(toRadians(angle)), ringStackPosition.y - 3 * sin(toRadians(angle)), angle - 90);
            chassis.addTargetPosition(ringTakeInStartPosition, NOTHING);
            chassis.addTargetPosition(ringTakeInEndPosition, TAKE_IN);
            chassis.addTargetPosition(chassis.getHighGoalEjectPosition(ringTakeInEndPosition), HIGH_GOAL);

            if (ringCase == "Quad") {
                ringTakeInStartPosition = new Position(ringStackPosition.x, ringStackPosition.y, angle - 90);
                ringTakeInEndPosition = new Position(ringStackPosition.x + 10 * cos(toRadians(angle)), ringStackPosition.y + 10 * sin(toRadians(angle)), angle - 90);
                chassis.addTargetPosition(ringTakeInStartPosition, NOTHING);
                chassis.addTargetPosition(ringTakeInEndPosition, TAKE_IN);
                chassis.addTargetPosition(chassis.getHighGoalEjectPosition(ringTakeInEndPosition), HIGH_GOAL);
            }
        }
        chassis.addTargetPosition(targetZonePosition.get(ringCase), WOBBLE_GOAL);
        chassis.addTargetPosition(robotStopPosition, STOP);
        chassis.start();

        while (opModeIsActive()) {
            if (chassis.isStopped() && chassis.getMode() == STOP) {
                servoUpDown.setPosition(downIndex);
                servoDelivery.setPosition(deliveryUpIndex);
                servoCatch.setPosition(pickCloseIndex);
                servoPush.setPosition(pushBackwardIndex);
                motorTakeIn.setPower(0);
                setEjectMotor(0);
            }
            else
                setEjectMotor(chassis.getHighGoalEjectSpeed());
            if (chassis.isStopped()) {
                switch (chassis.getMode()) {
                    case HIGH_GOAL: {
                        motorTakeIn.setPower(0);
                        servoUpDown.setPosition(upIndex);
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
            if (chassis.getMode() == TAKE_IN) {
                chassis.MAX_POWER = 1000;
                servoUpDown.setPosition(downIndex);
                motorTakeIn.setPower(-1);
            }
            else {
                chassis.MAX_POWER = 2000;
            }
            telemetry.addLine(chassis.getCurrentPosition().toString());
            telemetry.addData("mode", chassis.getMode());
            telemetry.addData("finishFlag", chassis.finishFlag);
            telemetry.update();
            idle();
            if (!opModeIsActive()) {
                webcam.closeCameraDevice();
                chassis.stop();
            }
        }

//        while (opModeIsActive()) {
//            idle();
//            telemetry.addData("", chassis.finishFlag);
//            telemetry.addData("", chassis.positionalPID.isFinishFlag());
//            telemetry.addData("", chassis.angularPID.isFinishFlag());
//            telemetry.addLine(chassis.getCurrentPosition().toString());
//            telemetry.update();
//            if (!opModeIsActive()) chassis.stop();
//        }
    }

    private void initMotorServo() {
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
}

