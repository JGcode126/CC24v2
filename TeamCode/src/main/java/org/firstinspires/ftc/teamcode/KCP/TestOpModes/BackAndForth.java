package org.firstinspires.ftc.teamcode.KCP.TestOpModes;

import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.ExamplePath;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.Stop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths;
import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.KCP.Movement;

@Autonomous(name = "Back and Forth")
public class BackAndForth extends BaseOpMode {

    Movement drive;
    AAA_Paths.Path state = ExamplePath;
    double currentV;

    @Override
    public void externalInit() {
        drive = new Movement(0,0,0);

        ExamplePath.compile();
        Stop.compile();


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
            case ExamplePath:
                forth();
                break;
            case Stop:
                stopMoving();
                break;
        }
    }


    public void forth(){
        double heading = 0;
        BaseOpMode.addData("%Done", ExamplePath.t);
        if(!drive.followPath(ExamplePath,1,heading, .7,true)){
            setState(Stop);
        }
    }

    public void stopMoving(){
        double heading = Math.PI;
        BaseOpMode.addData("%Done", Stop.t);
    }

    public void setState(AAA_Paths.Path state){
        this.state = state;
    }
}
