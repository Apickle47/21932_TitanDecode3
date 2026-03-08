package org.firstinspires.ftc.teamcode.subsystems;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.HashMap;

public class Hood {

    private Servo hood;
    private double x, y;

    public static boolean tracking = false;
    private double pos;

    public static double minHood = 1, maxHood = .35, closeHood = .6, farHood = .45;
    private static Pose pose;
    public static boolean TEST = false;
    public static double dist;
    public static double yDistToGoal = 0.8636;
    public static double trajAngle = Math.toRadians(0);

    public Hood(HardwareMap hardwareMap, HashMap<String, String> config, Pose startPos) {
        pose = startPos;
        hood = hardwareMap.get(Servo.class, config.get("hood"));
    }

    public void setHoodPosition(double pos) {
        this.pos = pos;
    }

    /*
    public void calcHoodPos(double dist) {
            pos = dist < 115 ? closeHood : farHood;
    }
    */
    public double getHoodPosition() {
        return pos;
    }

    public void hoodIncrement (double inc, double pos) {
        if (!(pos + inc > minHood) && !(pos + inc < maxHood)) {
            this.pos = pos + inc;
        }
        else if (pos + inc > minHood) {
            this.pos = minHood;
        }
        else {
            this.pos = maxHood;
        }
    }
/*
    public double calcHoodPos(double x) {
        return (((.4315 - 1)/(40.7 - 64)) * (Math.toDegrees(getLaunchAngle(x)) - 40.7)) + .4315;
    }

    public double getLaunchAngle(double x) {
        return Math.atan(2*yDistToGoal/x - Math.tan(trajAngle));
    }
*/
    public void update() {
        hood.setPosition(pos);
    }
}
