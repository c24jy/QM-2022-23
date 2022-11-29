package org.firstinspires.ftc.teamcode.openCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class camera extends OpenCvPipeline {
    Telemetry telemetry;
    SignalColor biggestArea = SignalColor.IDK;

    public camera(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    @Override
    public Mat processFrame(Mat input) {

//        converts rgb to hsv


        Scalar orangeLower = new Scalar(8, 114.5, 102);
        Scalar orangeUpper = new Scalar(15, 229.5, 256);
        Scalar purpleLower = new Scalar(125, 60, 76.5);
        Scalar purpleUpper = new Scalar(170, 204, 160);
        Scalar greenLower = new Scalar(23, 51, 76.5);
        Scalar greenUpper = new Scalar(47, 122, 200);

        double orangeArea = colorArea(input,orangeUpper,orangeLower,input);
        double purpleArea = colorArea(input,purpleUpper,purpleLower,input);
        double greenArea = colorArea(input,greenUpper,greenLower,input);


        if (orangeArea > purpleArea && orangeArea > greenArea){
            biggestArea = SignalColor.ORANGE;
        } else if (greenArea > orangeArea && greenArea > purpleArea) {
            biggestArea = SignalColor.GREEN;
        } else if (purpleArea > greenArea && purpleArea > orangeArea) {
            biggestArea = SignalColor.PURPLE;
        } else {
            biggestArea = SignalColor.IDK;
        }
                //
        // orange (h)8-13,   (s)45-90, (v) 40-90  - only for v need to check for larger range
        //purple (h)125-160  (s)30-80  (v) 30-60
        // green (h)23-47  (s) 20-40  (v) 30-60


        // orange (h)15-25,   (s)45-90, (v) 40-90  - only for v need to check for larger range
        //purple (h)249-320  (s)30-80  (v) 30-60
        // green (h)45-95  (s) 20-40  (v) 30-60


        telemetry.addData("Biggest Area", biggestArea);
        telemetry.update();


        return input;

    }

    public double colorArea(Mat mat, Scalar upper, Scalar lower, Mat input){
        Mat workingMat = new Mat();

        Imgproc.cvtColor(mat, workingMat, Imgproc.COLOR_RGB2HSV);

        //making pixels in the mat black and white based off a specific range - binary map of whats the color and whats not
        Core.inRange(workingMat, lower, upper, workingMat);

        //expand areas that are white - minimize the lone pixels, maximize the blobs (gets rid of some error)
        Imgproc.morphologyEx(workingMat,workingMat,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));

        //find contours

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(workingMat, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_TC89_KCOS);

        //biggest blob
        int index = 0;
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint newContour = contours.get(i);
            Rect bound = Imgproc.boundingRect(newContour);

            //of a good blob size
            if(bound.width > 10 && bound.height >10) {
                if (Imgproc.contourArea(newContour) > Imgproc.contourArea(contours.get(index))) {
                    index = i;
                }
            }
        }

        //done with this mat!!!
        workingMat.release();

        try{
            Imgproc.drawContours(input, contours, index, new Scalar(0,0,0));
            return Imgproc.contourArea(contours.get(index));
        } catch(Exception e){
            return 0;
        }

    }

    public SignalColor getBiggestArea() {
        return biggestArea;
    }
}
