package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.HashMap;
import java.util.Objects;

public class TopSensor {
    NormalizedColorSensor topSensor;
    private String color;
    private float normRed, normGreen, normBlue;
    public int ct;
    public TopSensor(HardwareMap hwMap, HashMap<String, String> config) {
        topSensor = hwMap.get(NormalizedColorSensor.class, config.get("topSensor"));
        topSensor.setGain(4);
    }

    public void update() {
        NormalizedRGBA colors = topSensor.getNormalizedColors();

        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
/*
        telemetry.addLine("TOP SENSOR");
        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);
        telemetry.addData("Color: ", getColor());
*/
        if (((normGreen > normBlue && normGreen > normRed) && (.055 < normGreen && normGreen < .082)) || ((normGreen > normBlue && normGreen > normRed) && (normGreen < 0.081 && normGreen > 0.052))) {
            color = "GREEN";
        } else if ((.03 < normRed && normRed < .045) || (normRed < 0.044 && normRed > 0.035) || (normRed > .028 && normGreen < .038)) {
            color = "PURPLE";
        } else {
            color = "UNKNOWN";
        }

    }
    public String getColor() {
        return color;
    }
    public float getR() {return normRed;}
    public float getG() {return normGreen;}
    public float getB() {return normBlue;}

    public int hasBall() {
        return (!(Objects.equals(color, "UNKNOWN"))) ? 1 : 0;
    }

}
