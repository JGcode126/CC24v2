package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor.largestRect;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.teamcode.Vision.blueProcessor;
import org.firstinspires.ftc.teamcode.Vision.redProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
@Autonomous(name = "BlueA")

public class BlueAuto extends CuddleOpMode {

    // Declare Subsystems


    blueProcessor blueProcessor = new blueProcessor();
    private VisionPortal visionPortal;
    AprilTagProcessor aprilTagProcessor;
    private WebcamName webcam1;
    Size size = new Size(1280,720);



    public enum VisionState {
        RIGHT, MIDDLE, LEFT, NODETECT
    }
    VisionState currentVisionState = VisionState.NODETECT;

    public void onInit(){
        super.onInit();
        // Initialize Subsystems
        setOpMode(this);
        telemetry.update();
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(601.169,601.169,278.814,259.214)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                .addProcessor(blueProcessor)
                //.setLiveViewContainerId(portal2ViewId)
                .setCameraResolution(size)
                .build();
        FtcDashboard.getInstance().startCameraStream(blueProcessor, 0);

        visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();

        if (redProcessor.returnPos() == 1){
            currentVisionState = VisionState.LEFT;
        } else if (redProcessor.returnPos() == 2) {
            currentVisionState = VisionState.MIDDLE;
        } else if (redProcessor.returnPos() == 3) {
            currentVisionState = VisionState.RIGHT;
        } else {
            currentVisionState = VisionState.NODETECT;
        }
    }


    public void mainLoop() {
        super.mainLoop();

            //Run Auto

        multTelemetry.addData("current state", currentVisionState);
        switch (currentVisionState) {
            case LEFT:

                if(tasksQueued == false){
                    queue.addTask(new PointTask(new Waypoint(new Pose(0.0,609.6,0.0),0.5), ptpController));
                    queue.addTask(new PointTask(new Waypoint(new Pose(606.6,0.0,0.0),0.5), ptpController));
                }
                tasksQueued = true;


                //robot should park in the leftmost square
                break;

            case MIDDLE:

                if(tasksQueued == false){
                    queue.addTask(new PointTask(new Waypoint(new Pose(0.0,609.6,0.0),0.5), ptpController));
                }
                tasksQueued = true;

                //robot should park in the middle
                break;

            case RIGHT:

                if(tasksQueued == false){
                    queue.addTask(new PointTask(new Waypoint(new Pose(0.0,609.6,0.0),0.5), ptpController));
                    queue.addTask(new PointTask(new Waypoint(new Pose(-606.6,0.0,0.0),0.5), ptpController));
                }
                tasksQueued = true;

                //robot should park to the left
                break;

            case NODETECT:

                if(tasksQueued == false){
                    queue.addTask(new PointTask(new Waypoint(new Pose(0.0,609.6,0.0),0.5), ptpController));
                }
                tasksQueued = true;

                //robot should park in the middle so we have some chance of scoring
                break;
        }



        
    }
}
