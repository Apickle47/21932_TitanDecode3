package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15.41)
            .forwardZeroPowerAcceleration(-34.570219277669665)
            .lateralZeroPowerAcceleration(-53.15618100562531)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(PedroDashTuning.translational)
            .secondaryTranslationalPIDFCoefficients(PedroDashTuning.secondaryTranslational)
            .headingPIDFCoefficients(PedroDashTuning.heading)
            .secondaryHeadingPIDFCoefficients(PedroDashTuning.secondaryHeading)
            .drivePIDFCoefficients(PedroDashTuning.drive)
            .secondaryDrivePIDFCoefficients(PedroDashTuning.secondaryDrive)
            .holdPointHeadingScaling(0.6)
            .holdPointTranslationalScaling(1)
            .centripetalScaling(.00005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.2, 2);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();

    }
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.827)
            .strafePodX(-4.508)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            ;
    public static MecanumConstants driveConstants = new MecanumConstants(){}
            .maxPower(1)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(80.47464493879184)
            .yVelocity(63.02087258166215);



}
