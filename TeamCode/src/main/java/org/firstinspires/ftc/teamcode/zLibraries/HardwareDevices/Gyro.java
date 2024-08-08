package org.firstinspires.ftc.teamcode.zLibraries.HardwareDevices;

import static org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode.hardware;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.KCP.Localization.Location;

import java.util.ArrayList;

public class Gyro {

 SparkFunOTOS otos;

    public static ArrayList<Gyro> gyros = new ArrayList<>();
    private double wrappedHeading = 0;
    private double rawHeading = 0;
    private double offset = 0;
    double previousTime = 0;
    double previousChange = 0;

//Problem:
    //When Gyro reads -3.14 or 3.14 and goes over, it
    // goes to negative version of that which causes it to spin in a circle to reach what it was at before

    public Gyro(double startHeading){
        gyros.add(this);
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
//        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        otos = hardware.get(SparkFunOTOS.class, "sensor_otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-3.5, 5, 0);
        otos.setOffset(offset);
        otos.setLinearScalar(1.01951);
        otos.setAngularScalar(1.0);
        otos.calibrateImu();
        otos.resetTracking();
        otos.getPosition().y = 0;
        otos.getPosition().x = 0;
        SparkFunOTOS.Pose2D startPos = new SparkFunOTOS.Pose2D(0,0,startHeading);
        otos.setPosition(startPos);

        update();


    }


    public double getHeading(){
        return wrappedHeading;
    }


    //maybe make total heading
    public double getRawHeading(){
        return rawHeading;
    }

    public double getUnwrappedHeading(){
        return rawHeading - offset;
    }

    public void resetHeading(){
        BaseOpMode.addData("Raw", rawHeading);
        offset = rawHeading;
    }



    //In radians
    public void setCurrentHeading(double heading){
        SparkFunOTOS.Pose2D startPos = new SparkFunOTOS.Pose2D(Location.x(), Location.y(),heading);
        otos.setPosition(startPos);
    }

    public static void updateAngles(){
        for(Gyro g : gyros){
            g.update();
        }
    }

    public static void zeroAngles(){
        for(Gyro g : gyros){
            g.resetHeading();
        }
    }

    public double getVeryRawHeading(){
        return 0;
    }

    public double getRateOfChange(){
            double change = getHeading();
            double deltaTime = (System.currentTimeMillis() - previousTime) / 1000.0;
            double deltaChange = change - previousChange;
            double rateOfChange = deltaChange/deltaTime;
            previousTime = System.currentTimeMillis();
            previousChange = change;
            return Math.abs(rateOfChange);
    }

    public static void resetGyroList(){
        gyros.clear();
    }

    private void update(){
        rawHeading = otos.getPosition().h;

        wrappedHeading = wrapAngle(rawHeading - offset);
//        BaseOpMode.addData("offset", offset);

    }

    private double wrapAngle(double angle){
//        angle += 2 * Math.PI;

        while (angle > Math.PI){
            angle -= Math.PI;
        }

        while (angle < -Math.PI){
            angle += Math.PI;
        }



        return angle;
    }




//    private BHI260IMU imuConstructor(String deviceName){
//        BHI260IMU imu;
//        imu = BaseOpMode.hardware.get(BHI260IMU.class, deviceName);
//        BHI260IMU.Parameters IMUparameters;
//
//        IMUparameters = new BHI260IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
//                )
//        );
//        imu.initialize(IMUparameters);
//        return imu;
//    }
}
