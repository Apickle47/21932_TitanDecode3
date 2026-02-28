package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BottomSensor;
import org.firstinspires.ftc.teamcode.subsystems.MiddleSensor;
import org.firstinspires.ftc.teamcode.subsystems.Rail;
import org.firstinspires.ftc.teamcode.subsystems.TopSensor;
import org.firstinspires.ftc.teamcode.subsystems.Util;


@TeleOp
public class ColorSensorTest extends OpMode {

    Util util;
    BottomSensor Bsensor;
    MiddleSensor Msensor;
    TopSensor Tsensor;
    Rail rail;

    @Override
    public void init() {
        util = new Util();
        Bsensor = new BottomSensor(hardwareMap, util.deviceConf);
        Msensor = new MiddleSensor(hardwareMap, util.deviceConf);
        Tsensor = new TopSensor(hardwareMap, util.deviceConf);
        rail = new Rail(hardwareMap, util.deviceConf);
    }

    @Override
    public void loop() {
        Bsensor.update();
        Msensor.update();
        Tsensor.update();
        telemetry.addData("top sensor R:", Tsensor.getR());
        telemetry.addData("top sensor G:", Tsensor.getG());
        telemetry.addData("top sensor B:", Tsensor.getB());
        telemetry.addData("top color", Tsensor.getColor());
        telemetry.addLine();
        telemetry.addData("middle sensor R:", Msensor.getR());
        telemetry.addData("middle sensor G:", Msensor.getG());
        telemetry.addData("middle sensor B:", Msensor.getB());
        telemetry.addData("middle color", Msensor.getColor());
        telemetry.addLine();
        telemetry.addData("bottom sensor R:", Bsensor.getR());
        telemetry.addData("bottom sensor G:", Bsensor.getG());
        telemetry.addData("bottom sensor B:", Bsensor.getB());
        telemetry.addData("bottom color", Bsensor.getColor());
        telemetry.update();
    }


}
