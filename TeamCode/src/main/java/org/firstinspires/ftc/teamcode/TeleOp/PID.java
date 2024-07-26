package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class PID  extends LinearOpMode{

    DcMotorEx motor;
    Drivetrain dt;

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class,"motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        waitForStart();
        while (opModeIsActive()) {
            double power = PIDControl(100, motor.getCurrentPosition());
            motor.setPower(power);
        }
    }


    public double PIDControl(double refrence, double state) {
        double error = refrence - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;


        timer.reset();

        double output = (error + Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
}
