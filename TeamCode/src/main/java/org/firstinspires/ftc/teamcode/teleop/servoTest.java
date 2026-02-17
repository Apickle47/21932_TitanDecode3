package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class servoTest extends LinearOpMode {
    public static double change = 0.001;
    public static String servoName = "hood";


    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, servoName);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.dpad_right)
            {
                servo.setPosition(servo.getPosition() + change);
            }
            if (gamepad1.dpad_left)
            {
                servo.setPosition(servo.getPosition() - change);
            }
            if (gamepad1.dpad_up)
            {
                servo.setPosition(1);
            }
            if (gamepad1.dpad_down)
            {
                servo.setPosition(0);
            }
            if (gamepad1.a)
            {
                servo.setDirection(Servo.Direction.FORWARD);
            }
            if (gamepad1.b)
            {
                servo.setDirection(Servo.Direction.REVERSE);
            }
            telemetry.addData("Pos", servo.getPosition());
            telemetry.addData("Dir", servo.getDirection());

            telemetry.update();


        }
    }
}