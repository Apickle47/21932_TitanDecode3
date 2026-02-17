
package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Indexer {
    Util util = new Util();
    BottomSensor bottomSensor = new BottomSensor(hardwareMap, util.deviceConf);
    MiddleSensor middleSensor = new MiddleSensor(hardwareMap, util.deviceConf);
    TopSensor topSensor = new TopSensor(hardwareMap, util.deviceConf);
    Telemetry telemetry;
    public void PPG() {

    }
    public void PGP() {

    }
    public void GPP() {
//if intake PPG cant index so shoot first to clear in gate
    }
}
