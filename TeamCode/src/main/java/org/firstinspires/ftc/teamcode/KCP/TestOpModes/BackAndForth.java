package org.firstinspires.ftc.teamcode.KCP.TestOpModes;

import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.Back;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.Forth;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.Stop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths;
import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.KCP.Movement;
import org.opencv.core.Mat;

@Autonomous(name = "Back and Forth")
public class BackAndForth extends BaseOpMode {

    Movement drive;
    AAA_Paths.Path state = Forth;
    double currentV;
    double heading = 0;

    @Override
    public void externalInit() {

    drive = new Movement(0,0,0);

        Forth.compile();
        Back.compile();


        BaseOpMode.addData("X", TwoWheelOdometry.x());
        BaseOpMode.addData("Y", TwoWheelOdometry.y());
        BaseOpMode.addData("H", TwoWheelOdometry.heading());
    }

    @Override
    public void externalLoop() {

        stateMachine();
        BaseOpMode.addData("X", TwoWheelOdometry.x());
        BaseOpMode.addData("Y", TwoWheelOdometry.y());
        BaseOpMode.addData("H", TwoWheelOdometry.heading());
    }

    public void stateMachine(){
        switch (state){
            case Forth:
                forth();
                break;
            case Back:
                back();
                break;
            case Stop:;
                stop();
        }
    }


    public void forth(){
        BaseOpMode.addData("%Done", Forth.t);
        BaseOpMode.addData("STATE","FORTH");
        if(!drive.followPath(Forth,.3,0, 1,false)){
            setState(Back);
        }
    }

    public void back(){
        BaseOpMode.addData("%Done", Back.t);
        BaseOpMode.addData("STATE","BACK");
        if(!drive.followPath(Back,.3,0, 0.7,true)){
            setState(Stop);
        }
    }

    public void setState(AAA_Paths.Path state){
        this.state = state;
    }
}
