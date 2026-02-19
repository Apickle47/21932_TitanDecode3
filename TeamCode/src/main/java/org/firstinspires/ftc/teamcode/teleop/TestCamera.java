package org.firstinspires.ftc.teamcode.teleop;

import android.view.autofill.AutofillId;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.InfernoTower;
import org.firstinspires.ftc.teamcode.subsystems.Util;

@Configurable
@TeleOp
public class TestCamera extends OpMode {
    TelemetryManager telemetryM;
    InfernoTower webcam1;
    Util util;
    @Override
    public void init() {
        util = new Util();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        webcam1 = new InfernoTower(hardwareMap, util.deviceConf);
    }

    @Override
    public void loop() {
        webcam1.detectAprilTags();
        telemetryM.addData("webcam reading", webcam1.detectAprilTags());
        telemetry.addData("webcam reading", webcam1.detectAprilTags());
    }
}
