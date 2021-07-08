package org.firstinspires.ftc.teamcode.util;

import android.os.Build;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.PriorityQueue;

import androidx.annotation.RequiresApi;

public class FindRingPipeline extends OpenCvPipeline {
    private ArrayList<ConnectedComponent> components;
    private PriorityQueue<Ring> rings;
    public volatile boolean finishFlag;
    private static final int minWidth = 20;
    private static final int minHeight = 5;
    private static final Comparator<Ring> cmp = (e1, e2) -> sgn(e1.getDistance() - e2.getDistance());
    private int left, right, top, bottom;

    public void setFrame(int left, int right, int top, int bottom) {
        this.left = left;
        this.right = right;
        this.top = top;
        this.bottom = bottom;
    }

    public static int sgn(double n) {
        if (n > 0) return 1;
        else if (n == 0) return 0;
        else return -1;
    }

    private void bfs(Point point, Mat input) {
        if (!Color.isRingColor(input.get((int)point.y, (int)point.x))) return;

        for (ConnectedComponent cc : components) if (cc.contains(point)) return;

        components.get(components.size() - 1).add(point);

        if (point.x > 0) bfs(new Point(point.x - 1, point.y), input);
        if (point.x < input.cols() - 1) bfs(new Point(point.x + 1, point.y), input);
        if (point.y > 0) bfs(new Point(point.x, point.y - 1), input);
        if (point.y < input.rows() - 1) bfs(new Point(point.x, point.y + 1), input);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public Mat processFrame(Mat input)
    {
        input.convertTo(input, CvType.CV_8UC3);
        Mat temp = new Mat(input.size(), input.type());
        Imgproc.medianBlur(input, temp, 5);

        Imgproc.rectangle(input,
                new Point(left - 1, top - 1),
                new Point(right + 1, bottom + 1),
                new Scalar(Color.RED.toChannel4()),
                2);

        finishFlag = false;
        rings = new PriorityQueue<>(cmp);
        components = new ArrayList<>();
        components.add(new ConnectedComponent());

        if (right == 0) right = input.cols();
        if (bottom == 0) bottom = input.rows();

        int cols = (right - left) / minWidth;
        int rows = (bottom - top) / minHeight;

        for (int i = 0; i < cols; i++)
            for (int j = 0; j < rows; j++)
             {
                bfs(new Point(i * minWidth + minWidth / 2 + left, j * minHeight + minHeight / 2 + top), temp);
                if (!components.get(components.size() - 1).isEmpty())
                    components.add(new ConnectedComponent());
            }

        for (ConnectedComponent component : components) {
            Ring ring = new Ring(component.left(), component.right(), component.top(), component.bottom(), component.downMost());
            if (ring.getWidth() >= minWidth && ring.getHeight() >= minHeight
                    && ring.getRatio() > 0.5 && ring.getRatio() < 10)
                        rings.add(ring);
        }
        finishFlag = true;
        if (rings.isEmpty())
            Imgproc.putText(input,
                    "None",
                    new Point(input.cols()/2 - 20, input.rows()/2 + 60),
                    Imgproc.FONT_HERSHEY_COMPLEX,
                    0.5,
                    new Scalar(Color.GREEN.toChannel4()));
        else for (Ring ring : rings)
            ring.drawFrame(input);

        return input;
    }

    public PriorityQueue<Ring> getRings(){
        return rings;
    }
}