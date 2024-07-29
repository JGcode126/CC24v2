package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;



import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;


@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    //Declare Subsystems

    //Timer
ElapsedTime runtime = new ElapsedTime();
    Drivetrain dt;
    IMU gyro;
    Boolean driverOriented=true;
    Scoring scoring;
    ElapsedTime clawTimer = new ElapsedTime();




    @Override

    public void init() {
        //Set timer to 0
        runtime.reset();
        dt = new Drivetrain(hardwareMap);
        scoring = new Scoring(hardwareMap);

        gyro = hardwareMap.get(IMU.class, "imuA");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        gyro.initialize(parameters);
        //Code that runs when you hit init

    }

    @Override
    public void start(){
        scoring.armUp();
        gyro.resetYaw();
        scoring.open();
        //Code that runs when you hit start
    }

    @Override
    public void loop() {
        //Code that *LOOPS* after you hit start


        if(gamepad1.options){
            driverOriented=true;
            gyro.resetYaw();
        }
        if(gamepad1.share){driverOriented=false;}
        dt.driveDO(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.right_trigger, gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), driverOriented);
        telemetry.addData("time", scoring.getTime());
        if(gamepad1.left_bumper){
            telemetry.addData("Claw pos", "open");
            scoring.open();

            scoring.armUp();
        } else if (gamepad1.right_bumper){
            scoring.closed();
        } else if(gamepad1.square){
            scoring.armDown();
        }
        if(gamepad1.cross){
            scoring.setDuckSpinner(Scoring.SpinDuck.ON);
        }

        telemetry.addData("Gyro Heading", gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void stop(){
        //Code that runs when you hit stop

        telemetry.addData("Runtime", runtime);
        telemetry.update();

    }



}
