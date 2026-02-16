package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class Rail {
    private Servo rail;

    public static double INLINE = 0.06, INDEX = 0.80;
    private double position = INLINE;

    public Rail(HardwareMap hardwareMap, HashMap<String, String> config) {
        rail = hardwareMap.get(Servo.class, config.get("rail"));
    }

    public void setPosition(double railPosition){
        position = railPosition;
    }



    public void update() {
        rail.setPosition(position);
    }

    public double getPosition() {
        return rail.getPosition();
    }
}

