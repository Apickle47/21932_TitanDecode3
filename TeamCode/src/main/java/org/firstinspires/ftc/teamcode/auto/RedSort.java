package org.firstinspires.ftc.teamcode.auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BottomSensor;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.InfernoTower;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MiddleSensor;
import org.firstinspires.ftc.teamcode.subsystems.Mortar;
import org.firstinspires.ftc.teamcode.subsystems.Rail;
import org.firstinspires.ftc.teamcode.subsystems.Signal;
import org.firstinspires.ftc.teamcode.subsystems.TopSensor;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Util;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Objects;

@Autonomous
public class RedSort extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private int ballCount;
    private boolean shooting, intaking = false;

    Util util;
    Mortar shooter;
    Turret turret;
    Intake intake;
    Gate gate;
    Hood hood;
    Signal signal;
    Rail rail;
    BottomSensor bottomSensor;
    MiddleSensor middleSensor;
    TopSensor topSensor;
    InfernoTower camera;
    Indexer indexer;


    private int shooterTargetSpeed;
    private int launchCount, shootPoseCount, launchIf;
    private double sortID;
    private int count;
    private boolean indexing = false;


    public enum PathState {
        DRIVE_START_POS_SHOOT_POS, SHOOT_PRE, SPIKE_ONE, RETURN_SHOOT1, SHOOT_ONE, SET_UP2, SPIKE_TWO, RETURN_SHOOT2, SHOOT_TWO, SET_UP3, SPIKE_THREE, RETURN_SHOOT3, SHOOT_THREE, SET_UP_HUMAN, HUMAN, RETURN_SHOOT_HUMAN, GINTAKE_AWAY, GINTAKE, RETURN_SHOOT_GINTAKE, GINTAKE_SHOOT, GINTAKE_AWAY2, GINTAKE2, RETURN_SHOOT_GINTAKE2, GINTAKE_SHOOT2, GINTAKE_SETUP, DONE, IDLE_SHOOT, IDLE_GATE, IDLE_GATE2, HIT_GATE1, HIT_GATE2
    }

    PathState pathState;

    private final Pose startPose = new Pose(117.771,126.922, Math.toRadians(38.937));
    private final Pose scanShootPose = new Pose(85.8,75.24, Math.toRadians(90));
    private final Pose shootPose = new Pose(85.8,75.24, 0);
    private final Pose lastShootPose = new Pose(84,109, 0);
    private final Pose spike1 = new Pose(127.7, 84.14, Math.toRadians(0));
    private final Pose setUp2 = new Pose(95, 58.2, Math.toRadians(0));
    private final Pose spike2 = new Pose(120.5,58.2,Math.toRadians(0));
    private final Pose setUp3 = new Pose(95,35, Math.toRadians(0));
    private final Pose spike3 = new Pose(140, 35, Math.toRadians(0));
    private final Pose setUpH = new Pose(128, 52, Math.toRadians(-90));
    private final Pose humanPose = new Pose(130,7, Math.toRadians(-90));
    private final Pose gintakeAwayPose1 = new Pose(102.65, 60.4, 0);
    private final Pose gintakePose = new Pose(130.51, 59.58, 0.4560);  //1 degree = 0.01745329251994329576923690768489 rad
    private final Pose hitGate = new Pose(132.105, 70, Math.toRadians(-90));
    private final Pose hitGateRev = new Pose(132,60.6, Math.toRadians(-90));


    private PathChain driveStartPosShootPos, spikeOne, spikeTwo, spikeThree, returnToShoot1, returnToShoot2, returnToShoot3, setUpTwo, setUpThree, setUpHuman, human, returnShootHuman, gintakeAway, gintake, returnToShootGintake, gintakeSetup;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scanShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scanShootPose.getHeading(), 0.8)
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .addParametricCallback(.85, () -> follower.setMaxPower(0.3))
                .addParametricCallback(0.99, () -> follower.setMaxPower(0.8))
                .build();
        spikeOne = follower.pathBuilder()
                .addPath(new BezierLine(scanShootPose, spike1))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //.addPath(new BezierLine(spike1, hitGate))
                //.setLinearHeadingInterpolation(spike2.getHeading(), hitGate.getHeading())
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .build();

        returnToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(spike1, lastShootPose))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .addParametricCallback(.85, () -> follower.setMaxPower(0.3))
                .addParametricCallback(0.99, () -> follower.setMaxPower(1))
                .build();
        setUpTwo = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, setUp2))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .build();

        spikeTwo = follower.pathBuilder()
                .addPath(new BezierLine(setUp2,spike2))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(spike2, hitGateRev))
                .setLinearHeadingInterpolation(spike2.getHeading(), hitGateRev.getHeading())
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .addParametricCallback(.85, () -> follower.setMaxPower(0.3))
                .addParametricCallback(0.99, () -> follower.setMaxPower(1))
                .build();
        returnToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(hitGateRev, shootPose))
                .setLinearHeadingInterpolation(hitGateRev.getHeading(), shootPose.getHeading())
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .addParametricCallback(.85, () -> follower.setMaxPower(0.3))
                .addParametricCallback(0.99, () -> follower.setMaxPower(1))
                .build();
        setUpThree = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, setUp3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), setUp3.getHeading())
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .build();
        spikeThree = follower.pathBuilder()
                .addPath(new BezierLine(setUp3, spike3))
                .setLinearHeadingInterpolation(setUp3.getHeading(), spike3.getHeading())
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .build();
        returnToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3, shootPose))
                .setLinearHeadingInterpolation(spike3.getHeading(), shootPose.getHeading())
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .addParametricCallback(.85, () -> follower.setMaxPower(0.3))
                .addParametricCallback(0.99, () -> follower.setMaxPower(1))
                .build();
        setUpHuman = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, setUpH))
                .setLinearHeadingInterpolation(shootPose.getHeading(), setUpH.getHeading())
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .build();
        gintakeSetup = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, gintakeAwayPose1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gintakeAwayPose1.getHeading())
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .build();
        gintake = follower.pathBuilder()
                .addPath(new BezierLine(gintakeAwayPose1, gintakePose))
                .setLinearHeadingInterpolation(gintakeAwayPose1.getHeading(), gintakePose.getHeading())
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .addParametricCallback(.85, () -> follower.setMaxPower(0.25))
                .addParametricCallback(0.99, () -> follower.setMaxPower(1))
                .build();
        gintakeAway = follower.pathBuilder()
                .addPath(new BezierLine(gintakePose, gintakeAwayPose1))
                .setLinearHeadingInterpolation(gintakePose.getHeading(), gintakeAwayPose1.getHeading())
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .build();

        returnToShootGintake = follower.pathBuilder()
                .addPath(new BezierLine(gintakeAwayPose1, shootPose))
                .setLinearHeadingInterpolation(gintakeAwayPose1.getHeading(), shootPose.getHeading())
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .addParametricCallback(.85, () -> follower.setMaxPower(0.4))
                .addParametricCallback(0.99, () -> follower.setMaxPower(1))
                .build();

    }

    private void intakeBalls() {
        if(intaking) {
            if (ballCount == 0) {
                signal.setPosition(Signal.VIOLET);
                gate.setPosition(Gate.CLOSE);
                intake.setAllPower(1);

            } else if (ballCount == 1) {
                signal.setPosition(Signal.RED);
                gate.setPosition(Gate.CLOSE);
                intake.setAllPower(1);

            } else if (ballCount == 2) {
                signal.setPosition(Signal.YELLOW);
                gate.setPosition(Gate.CLOSE);
                intake.setRollerPower(0);
                intake.setIntakePower(1);

            } else if (ballCount == 3) {
                signal.setPosition(Signal.GREEN);
                gate.setPosition(Gate.CLOSE);
                intake.setAllPower(0);
            }
        } else if (!shooting) {
            intake.setAllPower(0);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        pathState = PathState.DRIVE_START_POS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        count = 0;
        double shootTime = 3;
        // TODO add in any other init mechanisms

        util = new Util();
        shooter = new Mortar(hardwareMap, util.deviceConf);
        turret = new Turret(hardwareMap, util.deviceConf, new Pose(117.771,126.922,Math.toRadians(40.937)));
        intake = new Intake(hardwareMap, util.deviceConf);
        gate = new Gate(hardwareMap, util.deviceConf);
        hood = new Hood(hardwareMap, util.deviceConf, new Pose(117.771,126.922,Math.toRadians(38.937)));
        signal = new Signal(hardwareMap, util.deviceConf);
        rail = new Rail(hardwareMap, util.deviceConf);
        bottomSensor = new BottomSensor(hardwareMap, util.deviceConf);
        middleSensor = new MiddleSensor(hardwareMap, util.deviceConf);
        topSensor = new TopSensor(hardwareMap, util.deviceConf);
        camera = new InfernoTower(hardwareMap, util.deviceConf);
        indexer = new Indexer(hardwareMap, util.deviceConf);

        turret.setBasketPos(Turret.redBasket);
        follower.setPose(startPose);
        buildPaths();

        StateMachine machine = new StateMachineBuilder()
                .state(PathState.DRIVE_START_POS_SHOOT_POS)
                .onEnter( () -> {
                    follower.followPath(driveStartPosShootPos, true);
                })
                .loop( () -> {
                    gate.setPosition(Gate.OPEN);
                    count = -1;
                    sortID = camera.getLatestDetections();
                })
                .transition( () -> !follower.isBusy() && follower.atPose(scanShootPose, 0.5, 0.5, Math.toRadians(5)), PathState.SHOOT_PRE)

                .state(PathState.SHOOT_PRE)
                .loop( () -> {
                    shooting = true;
                    intaking = false;
                    intake.setAllPower(1);
                })
                .transitionTimed(2.5, PathState.SET_UP2)

                .state(PathState.SET_UP2)
                .onEnter( () -> {
                    follower.followPath(setUpTwo, true);
                })
                .loop( () -> {
                    gate.setPosition(Gate.CLOSE);
                    intaking = true;
                })
                .transition( () -> follower.atPose(setUp2, 0.5, 0.5, Math.toRadians(3)))

                .state(PathState.SPIKE_TWO)
                .onEnter( () -> {
                    follower.followPath(spikeTwo, true);
                })
                .loop( () -> {
                    intaking = true;
                })
                .transition( () -> follower.atPose(spike2, 0.5, 0.5, Math.toRadians(3)) || ballCount == 3, PathState.RETURN_SHOOT2)

                .state(PathState.RETURN_SHOOT2)
                .onEnter( () -> {
                    intaking = false;
                    shooting = true;
                    follower.followPath(returnToShoot2, true);
                })
                .loop( () -> {
                    intake.setAllPower(0);
                    switch((int)sortID) {
                        case(21):
                            rail.setPosition(Rail.INDEX);
                            break;
                        case(22):
                            break;
                        case(23):
                            break;
                        default:
                            break;
                    }
                    gate.setPosition(Gate.OPEN);
                })
                .transition( () -> follower.atPose(shootPose, 0.5, 0.5, Math.toRadians(3)), PathState.SHOOT_TWO)

                .state(PathState.SHOOT_TWO)
                .onEnter( () -> {
                    shooting = true;
                })
                .loop( () -> {
                    if (ballCount >= 1) {
                        intake.setAllPower(0.4);
                    }
                    if (ballCount == 0 && (int)sortID == 21) {
                        intake.setAllPower(0);
                        rail.setPosition(Rail.INLINE);
                    }
                })
                .transitionTimed(shootTime, PathState.SET_UP3)



                .state(PathState.SET_UP3)
                .onEnter( () -> {
                    shooting=false;
                    follower.followPath(setUpThree, true);
                })
                .loop( () -> {
                    gate.setPosition(Gate.CLOSE);
                    intaking=true;
                })
                .transition( () -> follower.atPose(setUp3, 0.5, 0.5, Math.toRadians(3)), PathState.SPIKE_THREE)

                .state(PathState.SPIKE_THREE)
                .onEnter( () -> {
                    follower.followPath(spikeThree, true);
                })
                .loop( () -> {
                    intaking = true;
                })
                .transition( () -> follower.atPose(spike3, 0.5, 0.5, Math.toRadians(3)) || ballCount == 3, PathState.RETURN_SHOOT3)

                .state(PathState.RETURN_SHOOT3)
                .onEnter( () -> {
                    intaking=false;
                    shooting=true;
                    follower.followPath(returnToShoot3, true);
                })
                .loop( () -> {
                    intake.setAllPower(0);
                    switch((int)sortID) {
                        case(21):
                            break;
                        case(22):
                            rail.setPosition(Rail.INDEX);
                            break;
                        case(23):
                            rail.setPosition(Rail.INDEX);
                            break;
                        default:
                            break;

                    }
                    gate.setPosition(Gate.OPEN);
                })
                .transition( () -> follower.atPose(shootPose, 0.5, 0.5, Math.toRadians(3)), PathState.SHOOT_THREE)


                .state(PathState.SHOOT_THREE)
                .onEnter( () -> {
                    shooting = true;
                })
                .loop( () -> {
                    intake.setRollerPower(0.4);
                    if (sortID == 21) {intake.setAllPower(0.4);}
                    if (ballCount == 1 && (int)sortID == 23) {
                        intake.setAllPower(1);
                        if(ballCount == 0) {
                            rail.setPosition(Rail.INLINE);
                        }
                    }
                    else if((int)sortID == 22 && ballCount == 1) {
                        intake.setAllPower(1);
                        if(ballCount == 0) {
                            rail.setPosition(Rail.INLINE);
                        }
                    }
                    else if (ballCount == 0) {
                        intake.setAllPower(1);
                    }
                })
                .transitionTimed(shootTime, PathState.SPIKE_ONE)



                .state(PathState.SPIKE_ONE)
                .onEnter( () -> {
                    shooting=false;
                    intaking=true;
                    follower.followPath(spikeOne, true);
                })
                .loop( () -> {
                    gate.setPosition(Gate.CLOSE);
                    intaking = true;
                })
                .transition( () -> follower.atPose(spike1, 0.5, 0.5, Math.toRadians(3)) || ballCount == 3, PathState.RETURN_SHOOT2)

                .state(PathState.RETURN_SHOOT1)
                .onEnter( () -> {
                    intaking=false;
                    follower.followPath(returnToShoot1, true);
                })
                .loop( () -> {
                    intake.setAllPower(0);
                    switch((int)sortID) {
                        case(21):
                            break;
                        case(22):
                            rail.setPosition(Rail.INDEX);
                            break;
                        case(23):
                            break;
                        default:
                            break;
                    }
                    gate.setPosition(Gate.OPEN);
                })
                .transition( () -> follower.atPose(lastShootPose, 0.5, 0.5, Math.toRadians(3)), PathState.SHOOT_ONE)

                .state(PathState.SHOOT_ONE)
                .onEnter( () -> {
                    shooting = true;
                })
                .loop( () -> {
                    intake.setAllPower(0.4);
                    if (ballCount == 0 && (int)sortID == 22) {
                        intake.setAllPower(1);
                        rail.setPosition(Rail.INLINE);
                    }
                })


                .build();



        waitForStart();
        machine.start();
        Turret.tracking = true;
        hood.setHoodPosition(.55);
        rail.setPosition(Rail.INLINE);
        //shooter.setVelocity(shooter.calcVelocity((71-20)*Math.sqrt(2)));
        shooter.setVelocity(1460);
        ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while(opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            follower.update();
            machine.update();
            intakeBalls();

            ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
            shooter.update();
            turret.update();
            intake.update();
            gate.update();
            hood.update();
            signal.update();
            rail.update();
            bottomSensor.update();
            middleSensor.update();
            topSensor.update();
            rail.update();

//TELEMETRY
            telemetry.addData("Latest AprilTag Reading", camera.getLatestDetections());
            telemetry.addData("path state", pathState.toString());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Gate Position", gate.getPosition());
            telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
            telemetry.addData("opModeTimer", opModeTimer.getElapsedTimeSeconds());
            //telemetry.addData("count", count);
            telemetry.addData("topSensor", topSensor.getColor());
            telemetry.addData("middleSensor", middleSensor.getColor());
            telemetry.addData("bottomSensor", bottomSensor.getColor());
            telemetry.addData("ball count", ballCount);

            telemetry.update();

        }
    }
}

