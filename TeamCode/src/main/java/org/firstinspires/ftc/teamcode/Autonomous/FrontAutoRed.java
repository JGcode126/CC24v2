package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.RedAutoFrontForward;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.RedAutoFrontGetRings;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.KCP.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;

@Autonomous(name="Front auto blue", group="Op mode")
public class FrontAutoRed extends BaseOpMode {
    AAA_Paths.Path path;
    Movement drive;
    Scoring scoring;
    ElapsedTime timer;
    @Override
    public void externalInit() {
        drive = new Movement(-6.3,109,Math.toRadians(90));
        path = RedAutoFrontForward;
        scoring = new Scoring(hardwareMap);
        timer = new ElapsedTime();

        RedAutoFrontForward.compile();
        RedAutoFrontGetRings.compile();
    }

    @Override
    public void externalInitLoop(){
        scoring.setArmState(Scoring.ArmSwitchStatement.UPCLOSED);
    }



    @Override
    public void externalStart(){

    }


    @Override
    public void externalLoop() {
        stateMachine();
        scoring.arm();
        BaseOpMode.addData("x",drive.getX());
        BaseOpMode.addData("y",drive.getY());
        BaseOpMode.addData("heading", Math.toDegrees(drive.getHeading()));
        BaseOpMode.addData("armState", scoring.getState());
    }
    public void stateMachine(){
        switch (path) {
            case RedAutoFrontForward:
                driveForward();
                break;
            case RedAutoFrontGetRings:
                getRings();
                break;
        }
    }
    public void driveForward(){
        scoring.setArmState(Scoring.ArmSwitchStatement.DOWNCLOSED);
        if(!drive.followPath(RedAutoFrontForward,0.6, Math.toRadians(135),.85, false)){
            if(scoring.clawColorYellow()){
                scoring.setArmState(Scoring.ArmSwitchStatement.DOWNOPEN);
                timer.reset();
                if (timer.seconds()>1) {
                    setState(RedAutoFrontGetRings);
                }
            } else {
                drive.holdPosition(-63,88,Math.toRadians(135));
            }

        }
    }
    public void getRings(){
        if(!drive.followPath(RedAutoFrontGetRings, 0.3,Math.toRadians(0),1,true) && scoring.getBeam());
    }
    public void setState(AAA_Paths.Path state) {
        path = state;
        timer.reset();
    }
}

