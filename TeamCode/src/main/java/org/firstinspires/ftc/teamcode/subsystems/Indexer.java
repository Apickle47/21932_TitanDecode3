
package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class Indexer {
    Util util = new Util();
    Rail rail = new Rail(hardwareMap, util.deviceConf);
    BottomSensor bottomSensor = new BottomSensor(hardwareMap, util.deviceConf);
    MiddleSensor middleSensor = new MiddleSensor(hardwareMap, util.deviceConf);
    TopSensor topSensor = new TopSensor(hardwareMap, util.deviceConf);
    Intake intake = new Intake(hardwareMap, util.deviceConf);
    Telemetry telemetry;
    Timer timer = new Timer();
    Timer timer1 = new Timer();
    public void PPG() {
        //PGP
        if (Objects.equals(bottomSensor.getColor(), "GREEN")) {
            intake.setRollerPower(.01);
            timer.resetTimer();
            if (timer.getElapsedTime() >= 100) {
                intake.setRollerPower(0);
                rail.setPosition(Rail.INDEX);
            }


        }
    }
    public void PGP() {

    }
    public void GPP() {
//if intake PPG cant index so shoot first to clear in gate
    }
}
