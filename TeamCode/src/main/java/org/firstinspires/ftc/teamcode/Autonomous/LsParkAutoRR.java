package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Config
@Autonomous(name = "LsAutoPark", group = "Partial Autonomous")
public class LsParkAutoRR extends LinearOpMode{


    // arm lift class
    public class ArmLift {
        private DcMotorEx armLift;

        public ArmLift(HardwareMap hardwareMap) {
            armLift = hardwareMap.get(DcMotorEx.class, "armLift");

            armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armLift.setDirection(DcMotorSimple.Direction.REVERSE);
            armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                armLift.setTargetPosition(50); // IMPORTANT: SET CORRECT POSITIONS
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
                armLift.setTargetPosition(150); // IMPORTANT: SET CORRECT POSITIONS
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        public class ArmPosDown implements Action {
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
                armLift.setTargetPosition(50); // IMPORTANT: SET CORRECT POSITIONS
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        public Action armPosDown() {
            return new ArmPosDown();
        }
    }

    public class ArmExtension {
        private DcMotorEx armExtension;

        public ArmExtension(HardwareMap hardwareMap) {
            armExtension = hardwareMap.get(DcMotorEx.class, "armExtension");
            armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armExtension.setDirection(DcMotorSimple.Direction.FORWARD);
            armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public class ExtendArm implements Action {
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
                armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            return new ArmExtension.ExtendArm();
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
                armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        public Action RetractArm() {
            return new ArmExtension.RetractArm();
        }
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
                claw.setPosition(0.9);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.2);
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
        Pose2d initialPose = new Pose2d(-25, -62, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a Claw instance
        Claw claw = new Claw(hardwareMap);
        // make an ArmLift instance
        ArmLift armLift = new ArmLift(hardwareMap);
        // make an ArmExtension instance
        ArmExtension armExtension = new ArmExtension(hardwareMap);

        TrajectoryActionBuilder autoParkTrajBuild = drive.actionBuilder(initialPose)
                // score preloaded specimen
                .strafeTo(new Vector2d(25, -60));

        Action autoParkTraj = autoParkTrajBuild.build();

        waitForStart();

        Actions.runBlocking(claw.closeClaw());


        if (isStopRequested()) return;

        Actions.runBlocking(
                // container for everything
                new SequentialAction(
                        autoParkTraj
                )
        );

    }

}
