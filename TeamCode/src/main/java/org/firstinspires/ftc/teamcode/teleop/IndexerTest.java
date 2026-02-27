package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BottomSensor;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.InfernoTower;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MiddleSensor;
import org.firstinspires.ftc.teamcode.subsystems.Rail;
import org.firstinspires.ftc.teamcode.subsystems.TopSensor;
import org.firstinspires.ftc.teamcode.subsystems.Util;

@TeleOp
public class IndexerTest extends LinearOpMode {
    private boolean shoots;

    Util util;
    Indexer indexer;
    Rail rail;
    BottomSensor bottomSensor;
    MiddleSensor middleSensor;
    TopSensor topSensor;
    Intake intake;
    TelemetryManager telemetryM;
    Timer timer1;
    Timer timer2;
    InfernoTower webcam1;

    @Override
    public void runOpMode() throws InterruptedException {

        util = new Util();
        indexer = new Indexer(hardwareMap, util.deviceConf);
        rail = new Rail(hardwareMap, util.deviceConf);
        bottomSensor = new BottomSensor(hardwareMap, util.deviceConf);
        middleSensor = new MiddleSensor(hardwareMap, util.deviceConf);
        topSensor = new TopSensor(hardwareMap, util.deviceConf);
        intake = new Intake(hardwareMap, util.deviceConf);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        timer1 = new Timer();
        timer2 = new Timer();
        webcam1 = new InfernoTower(hardwareMap, util.deviceConf);

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.leftBumperWasPressed()) {
                shoots = false;
            }
            if (gamepad1.rightBumperWasPressed()) {
                shoots = true;
            }

            if (gamepad1.aWasPressed()) {
                indexer.GPP(shoots);
            }
            if (gamepad1.bWasPressed()) {
                indexer.PGP(shoots);
            }
            if (gamepad1.xWasPressed()) {
                indexer.PPG(shoots);
            }



            telemetryM.addLine("COLOR SENSOR");
            telemetryM.addData("Bottom Sensor Color", bottomSensor.getColor());
            telemetryM.addData("Middle Sensor Color", middleSensor.getColor());
            telemetryM.addData("Top Sensor Color", topSensor.getColor());
            telemetryM.addData("Camera Reading", webcam1.detectAprilTags());

            telemetryM.update();
            telemetryM.update(telemetry);
        }
    }
}
