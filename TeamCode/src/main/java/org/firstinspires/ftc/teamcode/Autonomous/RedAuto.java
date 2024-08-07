package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor.largestRect;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.PIDController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.teamcode.Vision.redProcessor;
import org.firstinspires.ftc.vision.VisionPortal;


public class RedAuto extends LinearOpMode {

    // Declare Subsystems

    redProcessor redProcessor = new redProcessor();
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
                .addProcessor(redProcessor)
                //.setLiveViewContainerId(portal2ViewId)
                .setCameraResolution(size)
                .build();
        FtcDashboard.getInstance().startCameraStream(redProcessor, 0);

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

<<<<<<< Updated upstream
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


=======


            public class MotorPositionController {

                private DcMotor motor;

                private PIDController pidController;

                private ElapsedTime timer;

                private Position targetPosition;

                public MotorPositionController(DcMotor motor, double kp, double ki, double kd) {

                    this.motor = motor;

                    this.pidController = new PIDController(kp, ki, kd);

                    this.timer = new ElapsedTime();

                    this.targetPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);

                    this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                }

                public void setTargetPosition(double x, double y, double z) {

                    this.targetPosition = new Position(DistanceUnit.INCH, x, y, z, 0);

                }

                public void holdPosition() {

                    double error = calculateError();

                    double power = pidController.calculate(error, timer.seconds());

                    motor.setPower(power);

                    timer.reset();

                }

                private double calculateError() {

                    Position currentPosition = motor.getCurrentPosition(CurrentUnit.INCH);

                    double error = targetPosition.x - currentPosition.x;

                    return error;

                }

            }
>>>>>>> Stashed changes

        }
    }
}
