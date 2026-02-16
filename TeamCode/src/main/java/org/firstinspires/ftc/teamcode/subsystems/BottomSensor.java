package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class BottomSensor {
    NormalizedColorSensor bottomSensor;
    private String color;

    public BottomSensor(HardwareMap hwMap, HashMap<String, String> config) {
        bottomSensor = hwMap.get(NormalizedColorSensor.class, config.get("bottomSensor"));
        bottomSensor.setGain(4);
    }

    public void getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = bottomSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addLine("BOTTOM SENSOR");
        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);
        telemetry.addData("Color: ", getColor());

        if ((normGreen > normBlue && normGreen > normRed) && (.07 < normGreen && normGreen < .08)) {
            color = "GREEN";
        }
        else if ((.04 < normRed && normRed < .045)) {
            color = "PURPLE";
        }
        else {
            color = "UNKNOWN";
        }
    }
    public String getColor() {
        return color;
    }
}
