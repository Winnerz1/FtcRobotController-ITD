/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This code aims to create a "field-centric" drive program.
 * By tracking the heading, the "forward" direction can be
 * maintained, allowing for directions relative to the driver
 * instead of the robot.
 */

@TeleOp(group="Linear OpMode")
public class TeleOpFc_Linear extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armLiftLeft = null;
    private DcMotor armLiftRight = null;
    private DcMotor viperSlide = null;
    private Servo clawServo = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // make sure to test motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        armLiftLeft = hardwareMap.get(DcMotor.class, "armLiftLeft");
        armLiftRight = hardwareMap.get(DcMotor.class, "armLiftRight");
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");
        clawServo = hardwareMap.get(Servo.class, "claw");

        armLiftLeft.setDirection(DcMotor.Direction.FORWARD);
        armLiftRight.setDirection(DcMotor.Direction.FORWARD);
        viperSlide.setDirection(DcMotor.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // arm initializations
        int armLiftTarget = 0;

        int leftEncoderTarget = 0;
        int rightEncoderTarget = 0;

        double armLiftPower = 0.7;

        //int armBottomLimit = armLift.getCurrentPosition();
        //int armUpperLimit; // set an arm upper limit

        armLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armLiftLeft.setTargetPosition(0);
        armLiftRight.setTargetPosition(0);

        armLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armLiftLeft.setPower(armLiftPower);
        armLiftRight.setPower(armLiftPower);

        // slide initializations
        int slideTarget = 0;
        double slidePower = 0.7;

        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        viperSlide.setTargetPosition(0);

        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viperSlide.setPower(armLiftPower);

        // claw pos init
        double clawServoPosition = 0.5;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // arm lift
            armLiftTarget += -gamepad2.left_stick_y*3;
            //armTarget = Math.max(armTarget, armBottomLimit+10);
            //armTarget = Math.min(armTarget, armUpperLimit-10); set arm upper limit

            if (gamepad2.left_stick_y < 0.1 && gamepad2.left_stick_y > -0.1) {
                armLiftLeft.setTargetPosition(armLiftLeft.getCurrentPosition());
                armLiftRight.setTargetPosition(armLiftRight.getCurrentPosition());
            }

            armLiftLeft.setTargetPosition(armLiftTarget);
            armLiftRight.setTargetPosition(-armLiftTarget);

            armLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // viperslide
            if (gamepad2.right_stick_y < 0.1 && gamepad2.right_stick_y > -0.1) {
                viperSlide.setTargetPosition(viperSlide.getCurrentPosition());
            }

            viperSlide.setTargetPosition(slideTarget);
            viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad2.y) { // all arm mechanisms stop/hold in place
                armLiftLeft.setTargetPosition(armLiftLeft.getCurrentPosition());
                armLiftRight.setTargetPosition(armLiftLeft.getCurrentPosition());
                viperSlide.setTargetPosition(armLiftLeft.getCurrentPosition());
            }

            // claw (servo)
            if (gamepad2.right_bumper)
                clawServoPosition = 0.5;

            if (gamepad2.left_bumper)
                clawServoPosition = 0.4;

            clawServo.setPosition(clawServoPosition);
            
            // drive code
            double max;

            double driveSpeedMultiplier = (gamepad1.left_bumper) ? 1 : 0.5;

            double y = -gamepad1.left_stick_y; // Y stick value is reversed
            double x = gamepad1.left_stick_x + gamepad1.right_trigger - gamepad1.left_trigger;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.y) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double leftFrontPower = ( (rotY + rotX + rx) / denominator ) * driveSpeedMultiplier;
            double leftBackPower = ( (rotY - rotX + rx) / denominator ) * driveSpeedMultiplier;
            double rightFrontPower = ( (rotY - rotX - rx) / denominator ) * driveSpeedMultiplier;
            double rightBackPower = ( (rotY + rotX - rx) / denominator ) * driveSpeedMultiplier;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your  motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels and mechanisms
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("","");
            telemetry.addData("Lift Target", armLiftTarget);
            telemetry.addData("Lift Position", (armLiftLeft.getCurrentPosition() + armLiftRight.getCurrentPosition()) / 2);
            telemetry.addData("Lift Target", slideTarget);
            telemetry.addData("Slide Position", viperSlide.getCurrentPosition());
            telemetry.addData("Servo Target", clawServoPosition);
            telemetry.update();
        }
    }
}