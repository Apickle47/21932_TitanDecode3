package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.HashMap;

public class Tilt {
    private CRServo tilt1;
    private CRServo tilt2;
    private double power;

    public Tilt(HardwareMap hardwareMap, HashMap<String, String> config) {
        tilt1 = hardwareMap.get(CRServo.class, config.get("tilt1"));
        tilt2 = hardwareMap.get(CRServo.class, config.get("tilt2"));
    }

    public void tiltSetPower(double servoPower) {
        power = servoPower;
    }

    public double tiltGetPower() {
        return power;
    }

    public void update() {
        tilt1.setPower(power);
        tilt2.setPower(power);
    }
}
