package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    //Declare Subsystems

    //Timer
    ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;

    @Override
    public void init() {
        //Set timer to 0
        runtime.reset();
        //Code that runs when you hit init
        drivetrain = new Drivetrain(hardwareMap);
    }

    @Override
    public void start(){
        //Code that runs when you hit start
    }

    @Override
    public void loop() {
        //Code that *LOOPS* after you hit start
        drivetrain.drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, 0.5);
    }

    @Override
    public void stop(){
        //Code that runs when you hit stop

        telemetry.addData("Runtime", runtime);
        telemetry.update();

    }

}
