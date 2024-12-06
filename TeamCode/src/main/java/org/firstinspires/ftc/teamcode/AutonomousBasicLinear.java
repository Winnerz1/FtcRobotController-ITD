package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="LinearAutonomous", group="Linear OpMode")
public class AutonomousBasicLinear extends LinearOpMode {
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor armLift;
    private DcMotor armExtension;
    private Servo clawEnclosure;

    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        armLift = hardwareMap.get(DcMotor.class, "armLift");
        armExtension = hardwareMap.get(DcMotor.class, "armExtension");
        clawEnclosure = hardwareMap.get(Servo.class, "claw");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        armLift.setDirection(DcMotor.Direction.FORWARD);
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        int armTarget = armLift.getCurrentPosition();
//        int armBottomLimit = armLift.getCurrentPosition();
//        double armLiftPower = 0.8;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Started");
            telemetry.update();

            // moving forward
            leftFrontDrive.setPower(0.25);
            leftBackDrive.setPower(0.25);
            rightFrontDrive.setPower(0.25);
            rightBackDrive.setPower(0.25);
            sleep(2000);


            // strafing to the right to park in the observation zone
            leftFrontDrive.setPower(0.25);
            leftBackDrive.setPower(-0.25);
            rightFrontDrive.setPower(-0.25);
            rightBackDrive.setPower(0.25);
        }
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

//    // simple function to move the motors to drive the robot, used to simplify the code
//    public void drive(double motorPower) {
//        leftFrontDrive.setPower(motorPower);
//        leftBackDrive.setPower(motorPower);
//        rightFrontDrive.setPower(motorPower);
//        rightBackDrive.setPower(motorPower);
//    }
}
