
package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class Indexer {
    Util util = new Util();
    Rail rail = new Rail(hardwareMap, util.deviceConf);
    BottomSensor bottomSensor = new BottomSensor(hardwareMap, util.deviceConf);
    MiddleSensor middleSensor = new MiddleSensor(hardwareMap, util.deviceConf);
    TopSensor topSensor = new TopSensor(hardwareMap, util.deviceConf);
    Telemetry telemetry;
    public void PPG() {
        if(Objects.equals(topSensor.getColor(), "GREEN")) {
            rail.setPosition(Rail.INDEX);
        }
    }
    public void PGP() {

    }
    public void GPP() {
//if intake PPG cant index so shoot first to clear in gate
    }
}
