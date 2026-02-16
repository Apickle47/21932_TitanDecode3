package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class Signal {
    private Servo signal;

    public static double GREEN = 0.5, YELLOW = 0.388, RED = 0.28, VIOLET = 0.722;
    private double color = VIOLET;
 

    public Signal(HardwareMap hardwareMap, HashMap<String, String> config) {
        signal = hardwareMap.get(Servo.class, config.get("signal"));
    }

    public void setLEDColor(double color){
        this.color = color;
    }



    public void update() {
        signal.setPosition(color);
    }

    public double getLEDColor() {
        return signal.getPosition();
    }

    public void setPosition(double color) {
        signal.setPosition(color);
    }
}