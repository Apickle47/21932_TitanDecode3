package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class MiddleSensor {
    NormalizedColorSensor middleSensor;
    private String color;

    public MiddleSensor(HardwareMap hwMap, HashMap<String, String> config) {
        middleSensor = hwMap.get(NormalizedColorSensor.class, config.get("middleSensor"));
        middleSensor.setGain(4);
    }

    public void update() {
        NormalizedRGBA colors = middleSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

//        telemetry.addLine("MIDDLE SENSOR");
//        telemetry.addData("red", normRed);
//        telemetry.addData("green", normGreen);
//        telemetry.addData("blue", normBlue);
//        telemetry.addData("Color: ", getColor());

        if ((normGreen > normBlue && normGreen > normRed) && (.04 < normGreen && normGreen < .18)) {
            color = "GREEN";
        } else if ((normRed < .1)) {
            color = "PURPLE";
        } else {
            color = "UNKNOWN";
        }
    }
    public String getColor() {
        return color;
    }
}
