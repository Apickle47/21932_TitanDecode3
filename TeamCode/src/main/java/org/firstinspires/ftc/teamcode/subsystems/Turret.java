package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.teleop.dualServoTest.servoName2;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Base64;
import java.util.HashMap;

@Configurable
public class Turret {

    private ServoImplEx servoTurret;
    private ServoImplEx servoTurret2;
    private static Follower follower;
    //private PinpointLocalizer localizer;

    public static double maxRange = Math.toRadians(355) * 108.0/96;
    public static double x, y, heading, turretHeading, turretHeadingRelative;

    public static boolean tracking = false;

    private double pos;

    private Pose pose;

    public static Pose blueBasket = new Pose(0,144);
    public static Pose redBasket = new Pose(144, 144);
    public static Pose curBasket;

    public static double angleOffset = 0;

    public Turret(HardwareMap hardwareMap, HashMap<String, String> config, Pose startPos) {
        //drive = new MecanumDrive(hardwareMap, startPos); // TODO: set this to whatever position auton will end at
        follower = Constants.createFollower(hardwareMap);
        pose = startPos;
        servoTurret = hardwareMap.get(ServoImplEx.class, "turret");
        servoTurret2 = hardwareMap.get(ServoImplEx.class, "turret2");
        servoTurret.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servoTurret2.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public Pose getPose() {
        return pose;
    }
    public double getTurretHeading() {
        return turretHeading;
    }

    public double getTurretHeadingRelative() { return turretHeadingRelative; }

    public void setBasketPos(Pose pos) {
        curBasket = pos;
    }
    public Pose distanceToBasket() {
        return new Pose(curBasket.getY() - y, curBasket.getX() - x);
    }

    public void setPosition(double degrees) {
        pos = Math.toRadians(degrees);
    }

    public void resetRobotPose(Pose newPos) {
        follower.setPose(newPos);
    }

    public void setAngleOffset(double degrees) {
        angleOffset += Math.toRadians(degrees);
    }

    public void update() {

        pose = follower.getPose();
        x = follower.getPose().getX();
        y = follower.getPose().getY();
        heading = pose.getHeading();
        follower.update();

        turretHeadingRelative = Math.atan2(curBasket.getPose().getY() -y, curBasket.getPose().getX() -x);
        turretHeading = turretHeadingRelative - heading;

        turretHeading+=angleOffset;

        turretHeading %= 2*Math.PI;
        if(turretHeading>Math.PI) {
            turretHeading -= 2*Math.PI;
        }
        else if(turretHeading<-Math.PI) {
            turretHeading += 2*Math.PI;
        }
//        if(turretHeading < -Math.PI/2) {
//            turretHeading = -Math.PI/2;
//        }
//        if(turretHeading > Math.PI/2) {
//            turretHeading = Math.PI/2;
//        }

        if(tracking) {
            servoTurret.setPosition((turretHeading / (maxRange)) + .5);
            servoTurret2.setPosition((turretHeading / (maxRange)) + .5);
        }
        else {
            servoTurret.setPosition((pos/maxRange) + .5);
            servoTurret2.setPosition((pos/maxRange) + .5);
        }
    }
}
