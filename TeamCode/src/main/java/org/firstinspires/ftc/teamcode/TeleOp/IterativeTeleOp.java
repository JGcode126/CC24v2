package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    //Declare Subsystems

    //Timer
    ElapsedTime runtime = new ElapsedTime();
    Drivetrain dt;

    @Override
    public void init() {
        //Set timer to 0
        runtime.reset();
        dt = new Drivetrain(hardwareMap);
        //Code that runs ONCE when you hit init
    }

    @Override
    public void start(){
        //Code that runs ONCE when you hit start
    }

    @Override
    public void loop() {
        //Code that *LOOPS* after you hit start
        dt.drive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.right_trigger);
    }

    @Override
    public void stop(){
        //Code that runs when you hit stop

        telemetry.addData("Runtime", runtime);
        telemetry.update();

    }

}
