/*package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryActionFactory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Config
@Autonomous(name = "RsSpecimenAuto")
public class RsSpecimenAutoRR extends LinearOpMode{


    // arm lift class
    public class ArmLift {
        private DcMotorEx armLiftRight;
        private DcMotorEx armLiftLeft;

        public ArmLift(HardwareMap hardwareMap) {
            armLiftLeft = hardwareMap.get(DcMotorEx.class, "armLiftLeft");
            armLiftRight = hardwareMap.get(DcMotorEx.class, "armLiftRight");

            armLiftLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armLiftLeft.setDirection(DcMotorEx.Direction.REVERSE);
            armLiftRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            armLiftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            armLiftRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armLiftRight.setDirection(DcMotorEx.Direction.FORWARD);
            armLiftRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            armLiftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        public class ArmPosIdle implements Action {
            // checks if the arm lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armLiftLeft.setPower(0.8);
                    armLiftRight.setPower(0.8);
                    initialized = true;
                }

                // get current pos, set target pos
                double leftPos = armLiftLeft.getCurrentPosition();
                double rightPos = armLiftRight.getCurrentPosition();
                armLiftLeft.setTargetPosition(50); // IMPORTANT: SET CORRECT ARM POSITIONS
                armLiftRight.setTargetPosition(50);
                armLiftLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                armLiftRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                packet.put("armLiftLeft Pos:", leftPos);
                packet.put("armLiftRight Pos:", rightPos);
                if (leftPos < armLiftLeft.getTargetPosition() || rightPos < armLiftRight.getTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    return false;
                }
            }
        }

        public Action armPosIdle() {
            return new ArmPosIdle();
        }

        public class ArmPosPickup implements Action {
            // checks if the arm lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armLift.setPower(0.8);
                    initialized = true;
                }

                // get current pos, set target pos
                double pos = armLift.getCurrentPosition();
                armLift.setTargetPosition(175); // IMPORTANT: SET CORRECT POSITIONS
                armLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                packet.put("armLift Pos:", pos);
                if (pos < armLift.getTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    return false;
                }
            }
        }

        public Action armPosPickup() {
            return new ArmPosPickup();
        }

        public class ArmPosScore implements Action {
            // checks if the arm lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armLift.setPower(0.8);
                    initialized = true;
                }

                // get current pos, set target pos
                double pos = armLift.getCurrentPosition();
                armLift.setTargetPosition(332); // IMPORTANT: SET CORRECT POSITIONS
                armLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                packet.put("armLift Pos:", pos);
                if (pos < armLift.getTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    return false;
                }
            }
        }

        public Action armPosScore() {
            return new ArmPosScore();
        }

        public class ArmPosUp implements Action {
            // checks if the arm lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armLift.setPower(0.8);
                    initialized = true;
                }

                // get current pos, set target pos
                double pos = armLift.getCurrentPosition();
                armLift.setTargetPosition(390); // IMPORTANT: SET CORRECT POSITIONS
                armLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                packet.put("armLift Pos:", pos);
                if (pos < armLift.getTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    return false;
                }
            }
        }

        public Action armPosUp() {
            return new ArmPosUp();
        }
    }

    public class ArmExtension {
        private DcMotorEx armExtension;

        public ArmExtension(HardwareMap hardwareMap) {
            armExtension = hardwareMap.get(DcMotorEx.class, "armExtension");
            armExtension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armExtension.setDirection(DcMotorEx.Direction.FORWARD);
            armExtension.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            //armExtension.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        public class ExtendArm implements Action {
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armExtension.setPower(1);

                return false;
            }
        }

        public Action extendArm() {
            return new SequentialAction(new ExtendArm(), new SleepAction(6), new StopArm());
        }

        public class RetractArm implements Action {
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armExtension.setPower(-1);

                return false;
            }
        }

        public Action retractArm() {
            return new SequentialAction(new RetractArm(), new SleepAction(5), new StopArm());
        }

        public class StopArm implements Action {
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armExtension.setPower(0);

                return false;
            }
        }

        public Action stopArm() {
            return new StopArm();
        }

        /*public class ExtendArm implements Action {
            // checks if the arm lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armExtension.setPower(1);
                    initialized = true;
                }

                // get current pos, set target pos
                double pos = armExtension.getCurrentPosition();
                armExtension.setTargetPosition(25); // IMPORTANT: SET CORRECT POSITIONS
                armExtension.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                packet.put("armLift Pos:", pos);
                if (pos < armExtension.getTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    return false;
                }
            }
        }

        public Action extendArm() {
            return new ExtendArm();
        }

        public class RetractArm implements Action {
            // checks if the arm lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armExtension.setPower(1);
                    initialized = true;
                }

                // get current pos, set target pos
                double pos = armExtension.getCurrentPosition();
                armExtension.setTargetPosition(10); // IMPORTANT: SET CORRECT POSITIONS
                armExtension.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                packet.put("armLift Pos:", pos);
                if (pos > armExtension.getTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    return false;
                }
            }
        }

        public Action retractArm() {
            return new RetractArm();
        }*//*
    }

    // claw class
    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw"); // set correct claw servo name
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.45);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.7);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }
    }

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(5, -62, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a Claw instance
        Claw claw = new Claw(hardwareMap);
        // make an ArmLift instance
        ArmLift armLift = new ArmLift(hardwareMap);
        // make an ArmExtension instance
        ArmExtension armExtension = new ArmExtension(hardwareMap);

        TrajectoryActionBuilder scorePreloadTrajBuild = drive.actionBuilder(initialPose)
                // move forward a little to make room for arm lift
                .lineToY(-56)
                .waitSeconds(5)
                // score preloaded specimen
                .lineToY(-30);

        TrajectoryActionBuilder samplePushTrajBuild = scorePreloadTrajBuild.fresh()
                // push samples into observation zone
                .setTangent(Math.toRadians(310))
                .splineToConstantHeading(new Vector2d(36, -34), Math.PI / 2)
                .lineToY(-10)
                .setTangent(Math.toRadians(40))
                .splineToConstantHeading(new Vector2d(46, -50), Math.PI / 2)
                .lineToY(-10)
                .setTangent(Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(60, -50), Math.PI /2);

        TrajectoryActionBuilder specimenPickupTrajBuild = samplePushTrajBuild.fresh()
                // turn robot around, pick up specimen
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(38, -46, Math.toRadians(-90)), Math.PI)
                .setTangent(3*Math.PI/2)
                .lineToY(-62);

        TrajectoryActionBuilder specimenScoreTrajBuild = specimenPickupTrajBuild.fresh()
                // move to bar and score specimen
                .setTangent(Math.toRadians(130))
                .splineToLinearHeading(new Pose2d(8, -48, Math.toRadians(-275)), Math.toRadians(120))
                .setTangent(Math.PI/2)
                .lineToY(-30);

        TrajectoryActionBuilder parkTrajBuild = specimenScoreTrajBuild.fresh()
                .strafeTo(new Vector2d(60,-58));

        Action scorePreloadTraj = scorePreloadTrajBuild.build();
        Action samplePushTraj = samplePushTrajBuild.build();
        Action specimenPickupTraj = specimenPickupTrajBuild.build();
        Action specimenScoreTraj = specimenScoreTrajBuild.build();
        Action parkTraj = parkTrajBuild.build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                // container for everything
                new SequentialAction(
                        claw.closeClaw(),
                        armLift.armPosIdle(),
                        new ParallelAction(
                            scorePreloadTraj,
                            new ParallelAction(
                                armExtension.extendArm(),
                                armLift.armPosScore()
                            )
                        ),
                        claw.openClaw(),
                        armLift.armPosUp(),
                        new ParallelAction(
                            armExtension.retractArm(),
                            // push specimens into observation zone
                            samplePushTraj
                        ),
                        // cut off code past here if it doesn't work
                        new ParallelAction(
                            armLift.armPosPickup(),
                            specimenPickupTraj
                        ),
                        claw.closeClaw(),
                        armLift.armPosScore(),
                        new ParallelAction(
                            armExtension.extendArm(),
                            new SequentialAction(
                                new SleepAction(5.5),
                                specimenScoreTraj
                            )
                        ),
                        new ParallelAction(
                                armLift.armPosUp(),
                                claw.openClaw()
                        ),
                        new ParallelAction(
                                armExtension.retractArm(),
                                new SequentialAction(
                                        new SleepAction(5.5),
                                        parkTraj
                                )
                        )
                )
                // container for everything
        );

    }

}
*/