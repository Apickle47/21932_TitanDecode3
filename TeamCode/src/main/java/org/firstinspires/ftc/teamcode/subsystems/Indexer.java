
package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Objects;


public class Indexer {

    Util util;
    Rail rail;
    BottomSensor bottomSensor;
    MiddleSensor middleSensor;
    TopSensor topSensor;
    Intake intake;
    Timer timer1;
    Timer timer2;

    public Indexer(HardwareMap hardwareMap, HashMap<String, String> config) {
        util = new Util();
        rail = new Rail(hardwareMap, util.deviceConf);
        bottomSensor = new BottomSensor(hardwareMap, util.deviceConf);
        middleSensor = new MiddleSensor(hardwareMap, util.deviceConf);
        topSensor = new TopSensor(hardwareMap, util.deviceConf);
        intake = new Intake(hardwareMap, util.deviceConf);
        timer1 = new Timer();
        timer2 = new Timer();
    }

    public void PPG(boolean shoot) {
        String[] ramp = {"PURPLE", "PURPLE", "GREEN","PURPLE", "PURPLE", "GREEN","PURPLE", "PURPLE", "GREEN"};
        String[] shotBalls = {}; //TODO: add ramp and shotBalls to auto since it will get reset evey time
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
            if(timer1.getElapsedTime() > 500 && shoot) {
                intake.setAllPower(.7);
                timer1.resetTimer();
                if (timer1.getElapsedTime() >= 200) {
                    rail.setPosition(Rail.INLINE);
                }
            }

        }


    }
    public void PGP(boolean shoot) {
        //PPG
        if (Objects.equals(bottomSensor.getColor(), "GREEN")) {
            rail.setPosition(Rail.INDEX);
            timer1.resetTimer();
            if(timer1.getElapsedTime() > 500 && shoot) {
                intake.setAllPower(.7);
                timer1.resetTimer();
                if (timer1.getElapsedTime() >= 200) {
                    rail.setPosition(Rail.INLINE);
                }
            }
        }
        //GPP
        if (Objects.equals(topSensor.getColor(), "GREEN")) {
            rail.setPosition(Rail.INDEX);
            timer1.resetTimer();
            if(timer1.getElapsedTime() > 500) {
                intake.setRollerPower(.7);
            }
            if (timer1.getElapsedTime() >= 200) {
                intake.setAllPower(0);
                rail.setPosition(Rail.INLINE);
            }
            intake.setAllPower(0);
        }
    }
    public void GPP(boolean shoot) {
        //PGP
        rail.setPosition(Rail.INDEX);
        timer1.resetTimer();
        if(timer1.getElapsedTime() > 500 && shoot) {
            intake.setAllPower(1);
            timer1.resetTimer();
            if (timer1.getElapsedTime() >= 200) {
                rail.setPosition(Rail.INLINE);
            }
        }
    }

}
