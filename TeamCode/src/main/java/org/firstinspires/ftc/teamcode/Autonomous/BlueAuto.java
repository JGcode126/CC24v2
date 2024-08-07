package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor.largestRect;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.teamcode.Vision.blueProcessor;
import org.firstinspires.ftc.teamcode.Vision.redProcessor;
import org.firstinspires.ftc.vision.VisionPortal;


public class BlueAuto extends LinearOpMode {

    // Declare Subsystems

    blueProcessor blueProcessor = new blueProcessor();
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    Size size = new Size(1280,720);



    public enum VisionState {
        RIGHT, MIDDLE, LEFT, NODETECT
    }
    VisionState currentVisionState = VisionState.NODETECT;

    public void initialize(){
        // Initialize Subsystems
        setOpMode(this);
        telemetry.update();
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


    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if(opModeIsActive()){
            //Run Auto

            multTelemetry.addData("current state", currentVisionState);
            switch (currentVisionState) {
                case LEFT:
                    //robot should park in the leftmost square
                    break;

                case MIDDLE:
                    //robot should park in the middle
                    break;

                case RIGHT:
                    //robot should park to the left
                    break;

                case NODETECT:
                    //robot should park in the middle so we have some chance of scoring
                    break;
            }



        }
    }
}
