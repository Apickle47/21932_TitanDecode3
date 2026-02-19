package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;

import java.util.HashMap;

@Configurable
public class Util {

    public HashMap<String, String> deviceConf = new HashMap<String, String>();

    public Util() {
        // TODO: put proper names from config
        //             Name in code         Name in config
        deviceConf.put("frontLeftMotor",    "frontLeftMotor");//
        deviceConf.put("backLeftMotor",     "backLeftMotor");//
        deviceConf.put("frontRightMotor",   "frontRightMotor");//
        deviceConf.put("backRightMotor",    "backRightMotor");//
        deviceConf.put("intakeMotor",       "intake");//
        deviceConf.put("rollersMotor",      "roller");//
        //deviceConf.put("kicker",            "kicker");//
        //deviceConf.put("turretMotor",       "turret");
        deviceConf.put("webcam1",           "camera");
        deviceConf.put("shooter",           "flyMotor");//
        deviceConf.put("shooterTwo",        "flyMotor2");//
        deviceConf.put("gate",              "gate");//
        deviceConf.put("rail",              "rail");
        deviceConf.put("turret",            "turret");//
        deviceConf.put("turret2",           "turret2");//
        deviceConf.put("hood",              "hood");//
        deviceConf.put("signal",            "signal");
        deviceConf.put("bottomSensor",      "bottomSensor");
        deviceConf.put("middleSensor",      "middleSensor");
        deviceConf.put("topSensor",         "topSensor");
        deviceConf.put("tilt1",             "tilt1");
        deviceConf.put("tilt2",             "tilt2");
    }
}