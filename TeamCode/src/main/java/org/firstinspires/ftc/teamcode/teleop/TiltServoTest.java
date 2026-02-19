package org.firstinspires.ftc.teamcode.teleop;

import android.media.MediaPlayer;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Tilt;
import org.firstinspires.ftc.teamcode.subsystems.Util;

@Configurable
@TeleOp
public class TiltServoTest extends OpMode {
    Tilt tilt;
    Util util;
    @Override
    public void init() {
        util = new Util();
        tilt = new Tilt(hardwareMap, util.deviceConf);
    }

    @Override
    public void loop() {
        if(gamepad1.aWasPressed()) {
            tilt.tiltSetPower(1);
        }
        if(gamepad1.xWasPressed()) {
            tilt.tiltSetPower(-1);
        }
        if(gamepad1.bWasPressed()) {
            tilt.tiltSetPower(0);
        }
        telemetry.addData("tilt power", tilt.tiltGetPower());
        tilt.update();
    }
}
