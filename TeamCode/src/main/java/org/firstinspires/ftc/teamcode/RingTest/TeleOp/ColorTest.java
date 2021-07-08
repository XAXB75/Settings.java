package org.firstinspires.ftc.teamcode.RingTest.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Color;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static com.qualcomm.robotcore.util.Range.clip;

@TeleOp
@Disabled
public class ColorTest extends LinearOpMode {
    OpenCvCamera webcam;
    Point detectPoint = new Point();

    @Override
    public void runOpMode(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ColorTestPipeline pipeline = new ColorTestPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));

        telemetry.addLine("Not ready for start");
        telemetry.update();
        while (pipeline.RGB == null);
        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            detectPoint = new Point(detectPoint.x + 0.05 * gamepad1.left_stick_x, detectPoint.y + 0.05 * gamepad1.left_stick_y);
            if (gamepad1.right_trigger > 0.5) webcam.stopStreaming();
            if (gamepad1.right_bumper) {
                webcam.startStreaming(320,240);
                while (pipeline.RGB == null);
            }
            double RGB[] = pipeline.RGB;
            if (RGB == null) continue;
            telemetry.addData("R", RGB[0]);
            telemetry.addData("G", RGB[1]);
            telemetry.addData("B", RGB[2]);
            telemetry.update();
            idle();
        }
    }

    class ColorTestPipeline extends OpenCvPipeline {

        volatile double[] RGB;

        @Override
        public Mat processFrame(Mat input) {
            input.convertTo(input, CvType.CV_8UC3);

            detectPoint.x = clip(detectPoint.x, 1 ,input.cols() - 1);
            detectPoint.y = clip(detectPoint.y, 1 ,input.rows() - 1);
            RGB = input.get((int)detectPoint.y, (int)detectPoint.x);

            Imgproc.rectangle(
                    input,
                    new Point(detectPoint.x - 1, detectPoint.y - 1),
                    new Point(detectPoint.x + 1, detectPoint.y + 1),
                    new Scalar(Color.GREEN.toChannel4()),
                    -1
            );

            return input;
        }
    }
}
