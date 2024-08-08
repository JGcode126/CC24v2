package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.Back;

import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.BlueAutoFrontForward;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.BlueAutoFrontGetRings;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.Forth;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.Stop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KCP.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;

@Autonomous(name="Front auto blue", group="Op mode")
public class FrontAutoBlue extends BaseOpMode {
    AAA_Paths.Path path;
    Movement drive;
    Scoring scoring;
    @Override
    public void externalInit() {
        drive = new Movement(-6.3,109,Math.toRadians(90));
        path = BlueAutoFrontForward;
        scoring = new Scoring(hardwareMap);
        scoring.setArmState(Scoring.ArmSwitchStatement.UPCLOSED);

        BlueAutoFrontForward.compile();
        BlueAutoFrontGetRings.compile();
    }

    @Override
    public void externalInitLoop(){

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
    }
    public void stateMachine(){
        switch (path) {
            case BlueAutoFrontForward:
                driveForward();
                break;
            /*case BlueAutoFrontGetRings:
                getRings();*/
        }
    }
    public void driveForward(){

        if(!drive.followPath(BlueAutoFrontForward,0.6, Math.toRadians(135),1, true)){
            scoring.setArmState(Scoring.ArmSwitchStatement.DOWNOPEN);
            setState(BlueAutoFrontGetRings);

        }
    }
    public void getRings(){
        if(!drive.followPath(BlueAutoFrontGetRings, 0.3,Math.toRadians(0),1,true) && scoring.getBeam());
    }
    public void setState(AAA_Paths.Path state) {
        path = state;
    }
}

