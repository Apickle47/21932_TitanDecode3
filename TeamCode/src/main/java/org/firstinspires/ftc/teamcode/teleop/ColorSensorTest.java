package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BottomSensor;
import org.firstinspires.ftc.teamcode.subsystems.MiddleSensor;
import org.firstinspires.ftc.teamcode.subsystems.TopSensor;
import org.firstinspires.ftc.teamcode.subsystems.Util;

@TeleOp
public class ColorSensorTest extends OpMode {
    Util util = new Util();
    BottomSensor Bsensor = new BottomSensor(hardwareMap, util.deviceConf);
    MiddleSensor Msensor = new MiddleSensor(hardwareMap, util.deviceConf);
    TopSensor Tsensor = new TopSensor(hardwareMap, util.deviceConf);



    @Override
    public void init() {
    }

    @Override
    public void loop() {
        Bsensor.getDetectedColor(telemetry);
        Msensor.getDetectedColor(telemetry);
        Tsensor.findDetectedColor(telemetry);
    }
}
