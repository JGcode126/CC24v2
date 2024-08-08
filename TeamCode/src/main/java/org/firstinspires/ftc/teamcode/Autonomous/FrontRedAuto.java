package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.Park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KCP.Movement;
@Autonomous(name = "FrontRedAuto")
public class FrontRedAuto extends BaseOpMode{

    AAA_Paths.Path path;
    Movement drive;
    int conePos = 1;
    double heading = 0;

    @Override
    public void externalInit() {
        drive = new Movement(132, 106, Math.PI / 2.0);
        path = Park;
        setState(Park);

    }
    @Override
    public void externalInitLoop(){
    }

    @Override
    public void externalLoop() {
        stateMachine();
        drive.update();
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

    }
    public void Park() {
        if (conePos == 1) {
            drive.holdPosition(106, 84, Math.PI/2);
        } else if (conePos == 2) {

        } else if (conePos == 3) {

        }
    }
    public void MoveToCollect() {

    }
    public void MoveToScore() {

    }


}
