package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.Collect;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.MoveToCollect;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.MoveToScore;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.Park;
import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoreState.PCLOSE;
import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoreState.RCLOSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.KCP.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;

@Autonomous(name = "FrontRedAuto")
public class FrontRedAuto extends BaseOpMode{

    AAA_Paths.Path path;
    Movement drive;
    int conePos = 3;
    double heading = 0;
    Scoring scoring;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void externalInit() {
        drive = new Movement(132, 106, Math.PI / 2.0);
        path = Park;    
        setState(MoveToScore);
        scoring = new Scoring(hardwareMap);

    }
    @Override
    public void externalInitLoop(){
    }

    @Override
    public void externalLoop() {
        stateMachine();
        drive.update();
        scoring.scoring();
        BaseOpMode.addData("x",drive.getX());
        BaseOpMode.addData("y",drive.getY());
        BaseOpMode.addData("h",drive.getHeading());



    }

    public void stateMachine() {
        switch (path) {
            case Score:
                score();
                break;
            case Collect:
                Collect();
                break;
            case Park:
                Park();
                break;
            case MoveToCollect:
                MoveToCollect();
                break;
            case MoveToScore:
                MoveToScore();
                break;
        }
    }
    public void setState(AAA_Paths.Path state) {
        path = state;
    }
    public void score() {

    }
    public void Collect() {
        if (scoring.ring()) {
            if (timer.seconds() > 5) {
                scoring.setScoreState(RCLOSE);
                timer.reset();
                setState(MoveToScore);
            } else if (scoring.pixel()) {
                if (timer.seconds() > 5) {
                    scoring.setScoreState(PCLOSE);
                    timer.reset();
                    setState(MoveToScore);
                }
            } else {
                timer.reset();
            }
        }
    }

    public void Park() {
        if (conePos == 1) {
            drive.holdPosition(105, 82, Math.PI/2);
        } else if (conePos == 2) {
            drive.holdPosition(105, 102, Math.PI/2);
        } else if (conePos == 3) {
            drive.holdPosition(105, 125, Math.PI/2);
        }
    }
    public void MoveToCollect() {
        drive.holdPosition(100, 70, Math.PI/2);
        setState(Collect);

    }
    public void MoveToScore() {
        drive.holdPosition(105, 82, Math.PI/2);
    }


}
