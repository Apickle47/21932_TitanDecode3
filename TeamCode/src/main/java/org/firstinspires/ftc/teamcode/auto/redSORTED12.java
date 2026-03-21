package org.firstinspires.ftc.teamcode.auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.subsystems.ShooterTable;
import org.firstinspires.ftc.teamcode.subsystems.Signal;
import org.firstinspires.ftc.teamcode.subsystems.TopSensor;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Util;

import java.util.List;
import java.util.Objects;

@Autonomous(name = "Red SORTED 12")
public class redSORTED12 extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer, indexTimer;
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



    private double sortID = -1;
    private int offset;
    private double shooterTargetVel;


    public enum PathState {
        DRIVE_START_POS_SHOOT_POS, SHOOT_PRE, SPIKE_ONE, RETURN_SHOOT1, SHOOT_ONE, SPIKE_TWO, RETURN_SHOOT2, SHOOT_TWO, SPIKE_THREE, RETURN_SHOOT3, SHOOT_THREE, HIT_GATE1
    }

    PathState pathState;

    //1 degree = 0.01745329251994329576923690768489 rad
    //1 inch = 1 inch

    private final Pose startPose = new Pose(28.6712,127.9154, 2.4778).mirror();
    private final Pose scanShootPose = new Pose(86.8,78.24, Math.toRadians(90));
    private final Pose shootPose = new Pose(85.8,79.24, 0);
    private final Pose lastShootPose = new Pose(84,109, 0);
    private final Pose spike1 = new Pose(122.2, 84.14, Math.toRadians(0));
    private final Pose preGateHit = new Pose(110, 77.14, Math.toRadians(0));
    private final Pose gateHit = new Pose(17.233, 73.871, Math.toRadians(90)).mirror();
    private final Pose setUp2 = new Pose(93, 59.2, Math.toRadians(0));
    private final Pose leave2 = new Pose(105, 64.2, Math.toRadians(0));
    private final Pose spike2 = new Pose(124,58.2,Math.toRadians(0));
    private final Pose setUp3 = new Pose(90,36, Math.toRadians(0));
    private final Pose spike3 = new Pose(124, 35, Math.toRadians(0));


    private PathChain driveStartPosShootPos, spikeOne, hitGate, spikeTwo, spikeThree, returnToShoot1, returnToShoot2, returnToShoot3;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scanShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scanShootPose.getHeading())
                .addParametricCallback(.6, () -> follower.setMaxPower(0.9))
                .addParametricCallback(.7, () -> follower.setMaxPower(0.8))
                .addParametricCallback(.8, () -> follower.setMaxPower(0.7))
                .addParametricCallback(.9, () -> follower.setMaxPower(0.6))
                .addParametricCallback(.99, () -> follower.setMaxPower(1))
                .build();
        spikeOne = follower.pathBuilder()
                .addPath(new BezierLine(scanShootPose, spike1))
                .setConstantHeadingInterpolation(spike1.getHeading())
                .build();
        hitGate = follower.pathBuilder()
                .addPath(new BezierLine(spike1, preGateHit))
                .setLinearHeadingInterpolation(spike1.getHeading(), preGateHit.getHeading())
                .addPath(new BezierLine(preGateHit, gateHit))
                .setConstantHeadingInterpolation(gateHit.getHeading())
                .build();
        returnToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(gateHit, shootPose))
                .setLinearHeadingInterpolation(gateHit.getHeading(), shootPose.getHeading())
                .build();
        spikeTwo = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, setUp2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), setUp2.getHeading())
                .addPath(new BezierLine(setUp2,spike2))
                .setLinearHeadingInterpolation(setUp2.getHeading(), spike2.getHeading())
                .build();
        returnToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(spike2, leave2))
                .setLinearHeadingInterpolation(spike2.getHeading(), leave2.getHeading())
                .addPath(new BezierLine(leave2, shootPose))
                .setLinearHeadingInterpolation(leave2.getHeading(), shootPose.getHeading())
                .addParametricCallback(.5, () -> follower.setMaxPower(0.9))
                .addParametricCallback(.6, () -> follower.setMaxPower(0.8))
                .addParametricCallback(.7, () -> follower.setMaxPower(0.7))
                .addParametricCallback(.99, () -> follower.setMaxPower(1))
                //.addParametricCallback(.85, () -> follower.setMaxPower(0.25))
                //.addParametricCallback(0.99, () -> follower.setMaxPower(1))
                .build();
        spikeThree = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, setUp3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), setUp3.getHeading())
                .addPath(new BezierLine(setUp3, spike3))
                .setLinearHeadingInterpolation(setUp3.getHeading(), spike3.getHeading())
                .build();
        returnToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3, lastShootPose))
                .setLinearHeadingInterpolation(spike3.getHeading(), lastShootPose.getHeading())
                .addParametricCallback(.6, () -> follower.setMaxPower(0.9))
                .addParametricCallback(.7, () -> follower.setMaxPower(0.8))
                .addParametricCallback(.99, () -> follower.setMaxPower(1))
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
        indexTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        offset = 20;
        // TODO add in any other init mechanisms

        util = new Util();
        shooter = new Mortar(hardwareMap, util.deviceConf);
        turret = new Turret(hardwareMap, util.deviceConf, new Pose(28.6712,127.9154, 2.4778).mirror(), follower);
        intake = new Intake(hardwareMap, util.deviceConf);
        gate = new Gate(hardwareMap, util.deviceConf);
        hood = new Hood(hardwareMap, util.deviceConf, new Pose(28.6712,127.9154, 2.4778).mirror());
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
                    Turret.tracking = true;
                    follower.followPath(driveStartPosShootPos, true);
                    shooting=false;
                })
                .loop( () -> {
                    shooting = false;
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0.025, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm();
                    shooter.setVelocity(shooterTargetVel - offset - 10);
                })
                .onExit( () -> {
                })
                .transition( () -> follower.atPose(scanShootPose, 0.5, 0.5, Math.toRadians(4)), PathState.SHOOT_PRE)

                .state(PathState.SHOOT_PRE)
                .onEnter( () -> {
                    gate.setPosition(Gate.OPEN);
                    shooting = true;
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0.025, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm();
                    shooter.setVelocity(shooterTargetVel - offset - 10);
                })
                .loop( () -> {
                    shooting = true;
                    shooting = true;
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0.025, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm();
                    shooter.setVelocity(shooterTargetVel - offset - 10);
                    gate.setPosition(Gate.OPEN);
                    intake.setAllPower(1);
                    sortID = camera.getLatestDetections();
                })
                .onExit( () -> {
                    gate.setPosition(Gate.CLOSE);
                    rail.setPosition(Rail.INLINE);
                })
                .transitionTimed(1.5, PathState.SPIKE_ONE)

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
                .transition( () -> follower.atPose(spike1, 0.5, 0.5) || ballCount == 3, PathState.HIT_GATE1)

                .state(PathState.HIT_GATE1)
                .onEnter( () -> {
                    follower.followPath(hitGate, true);
                })
                .loop( () -> {
                    gate.setPosition(Gate.CLOSE);
                })
                .transitionTimed(2.5, PathState.RETURN_SHOOT1)

                .state(PathState.RETURN_SHOOT1)
                .onEnter( () -> {
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0.025, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm();
                    shooter.setVelocity(shooterTargetVel - offset);
                    follower.followPath(returnToShoot1, true);
                    intaking=false;
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
                .transition( () -> follower.atPose(shootPose, 0.5, 0.5, Math.toRadians(3)), PathState.SHOOT_ONE)


                .state(PathState.SHOOT_ONE)
                .onEnter( () -> {
                    shooting = true;
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0.025, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm();
                    shooter.setVelocity(shooterTargetVel - offset);
                })
                .loop( () -> {
                    shooting = true;
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0.025, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm();
                    shooter.setVelocity(shooterTargetVel - offset);
                    shooting = true;


                    if (sortID == 21) {
                        intake.setAllPower(1);
                    }


                    if (sortID == 22 && ballCount < 3) {
                        intake.setRollerPower(0.6);
                    }
                    if (ballCount == 1 && sortID == 22) {
                        intake.setAllPower(0.8);
                    }
                    if (ballCount == 0 && (int)sortID == 22 && rail.getPosition() != Rail.INLINE) {
                        intake.setAllPower(0);
                        rail.setPosition(Rail.INLINE + 0.01);
                        intake.setAllPower(0.6);
                    }


                    if (sortID == 23) {
                        intake.setAllPower(0.5);
                    }

                })
                .onExit( () -> {
                    rail.setPosition(Rail.INLINE);
                })
                .transitionTimed(3, PathState.SPIKE_TWO)





                .state(PathState.SPIKE_TWO)
                .onEnter( () -> {
                    shooting=false;
                    intaking=true;
                    follower.followPath(spikeTwo, true);
                })
                .loop( () -> {
                    gate.setPosition(Gate.CLOSE);
                    intaking = true;
                })
                .onExit( () -> {
                    gate.setPosition(Gate.OPEN);
                    intaking = false;
                })
                .transition( () -> follower.atPose(spike2, 0.5, 0.5) || ballCount == 3, PathState.RETURN_SHOOT2)

                .state(PathState.RETURN_SHOOT2)
                .onEnter( () -> {
                    gate.setPosition(Gate.OPEN);
                    follower.followPath(returnToShoot2, true);
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
                })
                .loop( () -> {
                    intake.setAllPower(0);
                    gate.setPosition(Gate.OPEN);
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
                })
                .transition( () -> follower.atPose(shootPose, 0.5, 0.5), PathState.SHOOT_TWO)


                .state(PathState.SHOOT_TWO)
                .onEnter( () -> {
                    intaking = false;
                    shooting = true;
                    gate.setPosition(Gate.OPEN);
                })
                .loop( () -> {
                    gate.setPosition(Gate.OPEN);
                    shooting = true;

                    if (ballCount == 2 && sortID == 21) {
                        intake.setRollerPower(0.6);
                    }
                    if (ballCount == 1 && sortID == 21) {
                        intake.setAllPower(0.8);
                    }
                    if (ballCount == 0 && (int)sortID == 21 && rail.getPosition() != Rail.INLINE) {
                        intake.setAllPower(0);
                        rail.setPosition(Rail.INLINE);
                        intake.setAllPower(1);
                    }


                    if (sortID == 22) {
                        intake.setAllPower(0.5);
                    }



                    if (sortID == 23) {
                        intake.setAllPower(1);
                    }


                })
                .onExit( () -> rail.setPosition(Rail.INLINE))
                .transitionTimed(3, PathState.SPIKE_THREE)

                .state(PathState.SPIKE_THREE)
                .onEnter( () -> {
                    shooting=false;
                    intaking=true;
                    follower.followPath(spikeThree, true);
                })
                .loop( () -> {
                    gate.setPosition(Gate.CLOSE);
                    intaking = true;
                })
                .transition( () -> follower.atPose(spike3, 0.5, 0.5) || ballCount == 3, PathState.RETURN_SHOOT3)
                .state(PathState.RETURN_SHOOT3)
                .onEnter( () -> {
                    follower.followPath(returnToShoot3, true);
                    intaking=false;
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
                })
                .loop( () -> {
                    intake.setAllPower(0);
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0.025, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm();
                    shooter.setVelocity(shooterTargetVel - offset);
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
                .transition( () -> follower.atPose(lastShootPose, 0.5, 0.5), PathState.SHOOT_THREE)

                .state(PathState.SHOOT_THREE)
                .onEnter( () -> {
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0.025, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm();
                    shooter.setVelocity(shooterTargetVel - offset);
                    shooting = true;
                })
                .loop( () -> {
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0.025, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm();
                    shooter.setVelocity(shooterTargetVel - offset);
                    shooting = true;
                    if (sortID == 21) {
                        intake.setRollerPower(0.6);
                    }
                    if (sortID == 21 && ballCount < 3) {
                        intake.setAllPower(0.7);
                    }


                    if (sortID == 22 && ballCount > 1) {
                        intake.setRollerPower(0.6);
                    }
                    if (sortID == 22 && ballCount < 2) {
                        intake.setAllPower(0);
                        indexTimer.resetTimer();
                        rail.setPosition(Rail.INLINE);
                        if (indexTimer.getElapsedTime() >= 600) {
                            intake.setAllPower(0.6);
                        }
                    }


                    if (sortID == 23) {
                        intake.setAllPower(0.7);
                    }
                    if (sortID == 23 && ballCount <= 1 && Objects.equals(topSensor.getColor(), "UNKNOWN")) {
                        intake.setAllPower(0);
                        indexTimer.resetTimer();
                        rail.setPosition(Rail.INLINE);
                        if (indexTimer.getElapsedTime() >= 600) {
                            intake.setAllPower(0.6);
                        }
                    }
                })
                .onExit( () -> {
                    rail.setPosition(Rail.INDEX);
                })

                .build();




        waitForStart();
        rail.setPosition(Rail.INLINE);
        ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
        machine.start();

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
            telemetry.addData("Latest AprilTag Reading", sortID);
            telemetry.addData("path state", pathState.toString());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Gate Position", gate.getPosition());
            telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
            telemetry.addData("opModeTimer", opModeTimer.getElapsedTimeSeconds());
            telemetry.addData("topSensor", topSensor.getColor());
            telemetry.addData("middleSensor", middleSensor.getColor());
            telemetry.addData("bottomSensor", bottomSensor.getColor());
            telemetry.addData("ball count", ballCount);

            telemetry.update();

        }
    }
}

