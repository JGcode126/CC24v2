package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.RedAuotoFrontDepositRings;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.RedAutoFrontForward;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.RedAutoFrontGetRings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.KCP.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.AutoDash;
import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Front auto red", group="Op mode")
public class FrontAutoRed extends BaseOpMode {
    AAA_Paths.Path path;
    Movement drive;
    Scoring scoring;
    ElapsedTime timer;
    boolean pathDone = false;
    private BasicVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    int parkPos;
    @Override
    public void externalInit() {
        drive = new Movement(-6.3,109,Math.toRadians(90));
        path = RedAutoFrontForward;
        scoring = new Scoring(hardwareMap);
        timer = new ElapsedTime();

        RedAutoFrontForward.compile();
        RedAutoFrontGetRings.compile();
        RedAuotoFrontDepositRings.compile();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionProcessor = new BasicVisionProcessor();

        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                .addProcessor(visionProcessor)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionProcessor, 0);
        waitForStart();
    }

    @Override
    public void externalInitLoop(){
        scoring.setArmState(Scoring.ArmSwitchStatement.UPCLOSED);
        parkPos=visionProcessor.whichThird();
    }



    @Override
    public void externalStart(){
        timer.reset();
    }


    @Override
    public void externalLoop() {
        stateMachine();
        scoring.arm();
        BaseOpMode.addData("x",drive.getX());
        BaseOpMode.addData("y",drive.getY());
        BaseOpMode.addData("heading", Math.toDegrees(drive.getHeading()));
        BaseOpMode.addData("armState", scoring.getState());
        BaseOpMode.addData("time", timer.seconds());
    }
    public void stateMachine(){
        switch (path) {
            case RedAutoFrontForward:
                driveForward();
                break;
            case RedAutoFrontGetRings:
                getRings();
                break;
            case RedAuotoFrontDepositRings:
                depositRing();
                break;
        }
    }
    public void driveForward(){
        scoring.setArmState(Scoring.ArmSwitchStatement.DOWNCLOSED);
        if (!pathDone) {
            if(!drive.followPath(RedAutoFrontForward, 0.6, Math.toRadians(135), 1, false)){
                pathDone = true;
            }
        } else {
            drive.holdPosition(AutoDash.depositX,AutoDash.depositY,Math.toRadians(135));
            if(scoring.clawDistClose() && scoring.clawColorYellow() || scoring.clawColorBlue()) {
                scoring.setArmState(Scoring.ArmSwitchStatement.DOWNOPEN);
                if (timer.seconds() > 4) {
                    setState(RedAutoFrontGetRings);
                }
            }
        }
    }
    public void getRings() {
        if (!pathDone) {
            if (!drive.followPath(RedAutoFrontGetRings, 0.6, Math.toRadians(90), 1, false)){
                pathDone = true;
            }
        } else{
            scoring.setArmState(Scoring.ArmSwitchStatement.UPOPEN);
            if(scoring.getBeam()){
                scoring.setArmState(Scoring.ArmSwitchStatement.UPCLOSED);
                setState(RedAuotoFrontDepositRings);
            }
        }
    }
    public void depositRing(){
        if(!pathDone){
            if(!drive.followPath(RedAuotoFrontDepositRings, .6, Math.toRadians(90),1,true));
            {
                pathDone = true;
                setState(RedAutoFrontGetRings);
            }
        }
    }
    public void setState(AAA_Paths.Path state) {
        path = state;
        timer.reset();
        pathDone = false;
    }
}

