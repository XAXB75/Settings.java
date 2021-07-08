package org.firstinspires.ftc.teamcode.RingTest.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.FindRingPipeline;
import org.firstinspires.ftc.teamcode.util.Ring;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
@Disabled
public class RingFindTest extends LinearOpMode
{
    OpenCvCamera webcam;

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Not ready for start");
        telemetry.update();

        FindRingPipeline pipeline = new FindRingPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));

        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        while (pipeline.getRings() == null);
        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            while (!pipeline.finishFlag);
            for (Ring ring : pipeline.getRings()) telemetry.addLine(ring.toString());
            telemetry.update();
            idle();
        }
    }
}
