package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.Back;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.BlueAutoForward;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.Forth;
import static org.firstinspires.ftc.teamcode.Autonomous.AAA_Paths.Path.Stop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KCP.Movement;

@Autonomous(name="Front auto blue", group="Op mode")
public class FrontAutoBlue extends BaseOpMode {
    AAA_Paths.Path path;
    Movement drive;
    @Override
    public void externalInit() {
        drive = new Movement(0,0,0);
    }

    @Override
    public void externalInitLoop(){

    }



    @Override
    public void externalStart(){

    }


    @Override
    public void externalLoop() {
        public void stateMachine(){
            switch (path) {
                case BlueAutoForward:

            }
        }

    }
}

