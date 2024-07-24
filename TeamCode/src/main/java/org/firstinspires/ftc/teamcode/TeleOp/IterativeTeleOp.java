package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import android.graphics.Point;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;

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

        //Code that runs when you hit init
    }

    @Override
    public void start(){
        //Code that runs when you hit start
        dt.drive(gamepad1.right_stick_y);
        dt.turn(gamepad1.left_stick_x);
        dt.strafe(gamepad1.right_stick_x);
    }

    @Override
    public void loop() {
        //Code that *LOOPS* after you hit start
        double drive = MathUtils.shift(gamepad1.right_stick_y, robot.imu.getAngle()).y;
        double strafe = MathUtils.shift(gamepad1.right_stick_x, robot.imu.getAngle()).x;
        double turn = gamepad1.left_stick_x;
    }

    @Override
    public void stop(){
        //Code that runs when you hit stop

        telemetry.addData("Runtime", runtime);
        telemetry.update();

    }

}
