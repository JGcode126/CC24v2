package org.firstinspires.ftc.teamcode.Utilities.DashConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDdash {
    public static  double Kpd = 0.025;
    public static  double Kdd = 0.025;
    public static double Kfd = 0.06;
    public static double DeadZone = 0.04;

    public static  double Kps = 0.15;
    public static  double Kds = 0.0;
    public static double Kfs = 0.095;

    public static  double Kph = -0.02;
    public static  double Kdh = 0.0;
    public static double Kih = 0.0;

}
