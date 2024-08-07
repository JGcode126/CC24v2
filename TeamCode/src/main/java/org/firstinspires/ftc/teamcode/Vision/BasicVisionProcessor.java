package org.firstinspires.ftc.teamcode.Vision;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_COMPLEX;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.rectangle;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
@Config
public class BasicVisionProcessor implements VisionProcessor, CameraStreamSource {

    public static int max_H = 160;
    public static int max_S = 230;
    public static int max_V = 230;

    public static Rect largestRect;

    public static int min_H = 80;
    public static int min_S = 100;
    public static int min_V = 130;

    //color data, using HSV colorspace, H=0-180, S=0-255, V=0-255
   /* public static int max_H = 200;
    public static int max_S = 255;
    public static int max_V = 255;

    public static Rect largestRect;
//blue
    public static int min_H = 80;
    public static int min_S = 70;
    public static int min_V = 100;*/

    //sets up for erode/dilate to get rid of stray pixels
    public static int erodeConstant = 1;
    public static int dilateConstant = 1;

    public Rect rectangleOutline;

    public static boolean targetDetected = false;


    private static int IMG_HEIGHT = 0;
    private static int IMG_WIDTH = 0;
    //sets up variables to collect image details

    private Mat output = new Mat(),
            modified = new Mat();
    private ArrayList<MatOfPoint> contours = new ArrayList<>();

    private Mat hierarchy = new Mat();
    //stuff for variables


    // colour settings to set up output image, you will see these colors if you choose to output camera stream to dashboard
    private Scalar orange = new Scalar(252, 186, 3);
    private Scalar lightBlue = new Scalar(3, 252, 227);

    //not currently used, will allow to write text
    // and the thickness variable can be used when drawing bounding rectangles or contours
    private int thickness = 10;
    private int font = FONT_HERSHEY_COMPLEX;

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    //this is for camera stream, don't fully understand yet



    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        input.copyTo(output);


        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();
        //just saving info

        //Scalar MIN_THRESH_PROP = new Scalar(0, 35, 38);
        //  Scalar MAX_THRESH_PROP = new Scalar(15, 100, 63);

        Scalar MIN_THRESH_PROP = new Scalar(min_H, min_S, min_V);
        Scalar MAX_THRESH_PROP = new Scalar(max_H, max_S, max_V);
        //setting up all the color thresholds


        // Imgproc.cvtColor(input, modified, COLOR_RGB2HSV);
        Imgproc.cvtColor(input, modified, COLOR_BGR2HSV);

        //goes from RGB to HSV color space
        //replace blue w/ red so it can use both sides of red color


        inRange(modified, MIN_THRESH_PROP, MAX_THRESH_PROP, modified);


        Rect submatRect = new Rect(new Point(4, 4), new Point(IMG_WIDTH, IMG_HEIGHT));
        modified = modified.submat(submatRect);
        //actual threshold thing to correct for top of screen being wierd and glitchy


//erode and dilate get rid of stray pixels and clean up data, can be made bigger
        erode(modified, modified, new Mat(erodeConstant, erodeConstant, CV_8U));
        //erode constant currently = 1, change to erode more or less
        dilate(modified, modified, new Mat(dilateConstant, dilateConstant, CV_8U));
        //dilate constant currently 1, should have erode/dilate be equal




        contours = new ArrayList<>();

        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        //figures out all the pixels on the edges of the blob, useful for finding center


        List<Rect> rects = new ArrayList<>();
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
        }//creates a bounding rectangle


        if (rects.size() != 0) {
//if anything is detected, find the biggest and use that
            this.largestRect = VisionUtils.sortRectsByMaxOption(1, VisionUtils.RECT_OPTION.AREA, rects).get(0);
//draws rectangle around biggest shape
            rectangle(output, largestRect, orange, 10); //, thickness);

            targetDetected = true;


        } else {
            targetDetected = false;
        }
        //draws contours around shapes
        drawContours(output, contours, -1, lightBlue);

        Bitmap b = Bitmap.createBitmap(modified.width(), modified.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(modified, b);
        lastFrame.set(b);


        return output;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//this is not used, can be used to draw on image but not really that useful, easier to just do in the main area
    }
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

}

