package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor.largestRect;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.teamcode.Vision.blueProcessor;
import org.firstinspires.ftc.teamcode.Vision.redProcessor;
import org.firstinspires.ftc.vision.VisionPortal;


public class RedAuto extends CuddleOpMode {

    // Declare Subsystems

    redProcessor redProcessor = new redProcessor();
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    Size size = new Size(1280, 720);


    public enum VisionState {
        RIGHT, MIDDLE, LEFT, NODETECT
    }

    VisionState currentVisionState = VisionState.NODETECT;

    public void initialize() {
        // Initialize Subsystems
        setOpMode(this);
        telemetry.update();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                .addProcessor(redProcessor)
                //.setLiveViewContainerId(portal2ViewId)
                .setCameraResolution(size)
                .build();
        FtcDashboard.getInstance().startCameraStream(redProcessor, 0);

        visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();

        if (redProcessor.returnPos() == 1) {
            currentVisionState = VisionState.LEFT;
        } else if (redProcessor.returnPos() == 2) {
            currentVisionState = VisionState.MIDDLE;
        } else if (redProcessor.returnPos() == 3) {
            currentVisionState = VisionState.RIGHT;
        } else {
            currentVisionState = VisionState.NODETECT;
        }
    }


    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if (opModeIsActive()) {
            //Run Auto

            multTelemetry.addData("current state", currentVisionState);
            switch (currentVisionState) {
                case LEFT:

                    if (tasksQueued == false) {
                        queue.addTask(new PointTask(new Waypoint(new Pose(0.0, 609.6, 0.0), 0.5), ptpController));
                        queue.addTask(new PointTask(new Waypoint(new Pose(606.6, 0.0, 0.0), 0.5), ptpController));
                    }
                    tasksQueued = true;


                    //robot should park in the leftmost square
                    break;

                case MIDDLE:

                    if (tasksQueued == false) {
                        queue.addTask(new PointTask(new Waypoint(new Pose(0.0, 609.6, 0.0), 0.5), ptpController));
                    }
                    tasksQueued = true;

                    //robot should park in the middle
                    break;

                case RIGHT:

                    if (tasksQueued == false) {
                        queue.addTask(new PointTask(new Waypoint(new Pose(0.0, 609.6, 0.0), 0.5), ptpController));
                        queue.addTask(new PointTask(new Waypoint(new Pose(-606.6, 0.0, 0.0), 0.5), ptpController));
                    }
                    tasksQueued = true;

                    //robot should park to the left
                    break;

                case NODETECT:

                    if (tasksQueued == false) {
                        queue.addTask(new PointTask(new Waypoint(new Pose(0.0, 609.6, 0.0), 0.5), ptpController));
                    }
                    tasksQueued = true;

                    //robot should park in the middle so we have some chance of scoring
                    break;
            }


        }
    }
}