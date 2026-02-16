package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BottomSensor;
import org.firstinspires.ftc.teamcode.subsystems.MiddleSensor;
import org.firstinspires.ftc.teamcode.subsystems.TopSensor;
import org.firstinspires.ftc.teamcode.subsystems.Util;

@TeleOp
public class ColorSensorTest extends OpMode {

    Util util;
    BottomSensor Bsensor;
    MiddleSensor Msensor;
    TopSensor Tsensor;

    @Override
    public void init() {
        util = new Util();
        Bsensor = new BottomSensor(hardwareMap, util.deviceConf);
        Msensor = new MiddleSensor(hardwareMap, util.deviceConf);
        Tsensor = new TopSensor(hardwareMap, util.deviceConf);
    }

    @Override
    public void loop() {
        Bsensor.update();
        Msensor.update();
        Tsensor.update();
    }

}
