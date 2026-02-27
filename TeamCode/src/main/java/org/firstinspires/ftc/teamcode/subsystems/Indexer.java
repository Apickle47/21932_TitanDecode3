
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
    Timer timer1 = new Timer();
    Timer timer2 = new Timer();

    public void PPG() {
        String[] ramp = {"PURPLE", "PURPLE", "GREEN","PURPLE", "PURPLE", "GREEN","PURPLE", "PURPLE", "GREEN"};
        String[] shotBalls = {};
        int shotBallCounter;
        boolean indexing;
//        //PGP
//        if (Objects.equals(middleSensor.getColor(), "GREEN")) {
//            timer.resetTimer();
//            intake.setRollerPower(1);
//            indexing = true;
//            if (timer.getElapsedTime() >= 100 && indexing) {
//                intake.setIntakePower(0);
//                rail.setPosition(Rail.INDEX);
//                indexing = false;
//            }
//            if (timer.getElapsedTime() >= 200 && !indexing) {
//                intake.setAllPower();
//            }
        //GPP
        if (Objects.equals(topSensor.getColor(), "GREEN")) {
            rail.setPosition(Rail.INDEX);
            timer1.resetTimer();
            if(timer1.getElapsedTime() > 500) {
                intake.setAllPower(1);
            }
            if (timer1.getElapsedTime() >= 200) {
                rail.setPosition(Rail.INLINE);
            }
        }


    }
    public void PGP() {
        //PPG
        if (Objects.equals(bottomSensor.getColor(), "GREEN")) {
            rail.setPosition(Rail.INDEX);
            timer1.resetTimer();
            if(timer1.getElapsedTime() > 500) {
                intake.setAllPower(1);
            }
            if (timer1.getElapsedTime() >= 200) {
                rail.setPosition(Rail.INLINE);
            }
        }
        //GPP
        if (Objects.equals(topSensor.getColor(), "GREEN")) {
            rail.setPosition(Rail.INDEX);
            timer1.resetTimer();
            if(timer1.getElapsedTime() > 500) {
                intake.setRollerPower(1);
            }
            if (timer1.getElapsedTime() >= 200) {
                intake.setAllPower(0);
                rail.setPosition(Rail.INLINE);
            }
            intake.setAllPower(0);
        }
    }
    public void GPP() {
        //PGP
        rail.setPosition(Rail.INDEX);
        timer1.resetTimer();
        if(timer1.getElapsedTime() > 500) {
            intake.setAllPower(1);
        }
        if (timer1.getElapsedTime() >= 200) {
            rail.setPosition(Rail.INLINE);
        }
    }
}
