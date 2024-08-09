package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Sensors.SparkFunOTOS;

@TeleOp(name = "Localization Test")

public class LocalizationTest extends OpMode {


    SparkFunOTOS myOtos;
    double x;
    double y;
    double h;


    public void init() {
        setOpMode(this);
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-3.5, 5, 0);
        myOtos.setOffset(offset);
        myOtos.setLinearScalar(1.01951);
        myOtos.setAngularScalar(1.00763666667);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, Math.PI / 2);
        myOtos.setPosition(currentPosition);
//        myOtos.getPosition().y = 0;
//        myOtos.getPosition().x = 0;
//        myOtos.getPosition().h = 0;


    }
    public void loop(){
        x = myOtos.getPosition().x;
        y = myOtos.getPosition().y;
        h = myOtos.getPosition().h;
        multTelemetry.addData("x", x);
        multTelemetry.addData("y", y);
        multTelemetry.addData("heading", h);
    }

}
