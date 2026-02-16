package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Objects;

public class TopSensor {
    NormalizedColorSensor topSensor;
    private String color;

    public TopSensor(HardwareMap hwMap, HashMap<String, String> config) {
        topSensor = hwMap.get(NormalizedColorSensor.class, config.get("topSensor"));
        topSensor.setGain(4);
    }

    public void update() {
        NormalizedRGBA colors = topSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

//        telemetry.addLine("TOP SENSOR");
//        telemetry.addData("red", normRed);
//        telemetry.addData("green", normGreen);
//        telemetry.addData("blue", normBlue);
//        telemetry.addData("Color: ", getColor());

        if ((normGreen > normBlue && normGreen > normRed) && (.055 < normGreen && normGreen < .086)) {
            color = "GREEN";
        } else if ((.03 < normRed && normRed < .045)) {
            color = "PURPLE";
        } else {
            color = "UNKNOWN";
        }
    }
    public String getColor() {
        return color;
    }

    public boolean hasBall() {
        return !(Objects.equals(color, "UNKNOWN"));
    }
}
