package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Chassis;
import org.firstinspires.ftc.teamcode.util.Position;

import static org.firstinspires.ftc.teamcode.util.Chassis.PURE_PURSUIT;
import static org.firstinspires.ftc.teamcode.util.Chassis.STOP;

@Autonomous
@Disabled
public class PurePursuitTest extends OpMode {
    Chassis chassis;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        chassis = new Chassis(hardwareMap,
                "m10", "m11", "m13", "m12",
                "m23", "m21", "m22");

        chassis.stop();
        chassis.reset();
        chassis.setCurrentPosition(new Position());


        new Thread(chassis).start();

        chassis.addTargetPosition(new Position(23, 69, 30), PURE_PURSUIT);
        chassis.addTargetPosition(new Position(0, 46, 60), PURE_PURSUIT);
        chassis.addTargetPosition(new Position(0,23,90),PURE_PURSUIT);
//        chassis.addTargetPosition(new Position(0, 69, 120), PURE_PURSUIT);
//        chassis.addTargetPosition(new Position(23,23,150),PURE_PURSUIT);
//        chassis.addTargetPosition(new Position(0,23,150),PURE_PURSUIT);

        chassis.addTargetPosition(chassis.getPreviousPosition(), STOP);
    }

    @Override
    public void loop() {
        chassis.start();
        telemetry.addLine(chassis.getCurrentPosition().toString());
        telemetry.addData("positional", chassis.positionalPID.finishFlag);
        telemetry.addData("angular", chassis.angularPID.finishFlag);
        telemetry.addData("index", chassis.getIndex());
        telemetry.addData("finish", chassis.finishFlag);
        telemetry.update();
    }

    @Override
    public void stop() {
        chassis.stop();
    }
}


