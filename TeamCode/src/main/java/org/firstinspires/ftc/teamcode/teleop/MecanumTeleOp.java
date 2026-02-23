package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    // Declare motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Declare servo
    private Servo testServo;

    @Override
    public void runOpMode() {

        // Connect motors to hardware map (must match names in Driver Hub config)
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        testServo = hardwareMap.get(Servo.class, "testServo");

        // Reverse left side motors so all wheels drive forward together
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Ready to run!");
        telemetry.update();

        waitForStart(); // Wait for the driver to press PLAY

        while (opModeIsActive()) {

            // Read gamepad inputs
            double axial    = -gamepad1.left_stick_y;  // Forward / backward
            double lateral  =  gamepad1.left_stick_x;  // Strafe left / right
            double yaw      =  gamepad1.right_stick_x; // Rotate

            // Calculate power for each wheel
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Scale down if any value exceeds 1.0
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Send power to motors
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Axon servo — right bumper to spin, stops when released
            if (gamepad1.right_bumper) {
                testServo.setPosition(1.0); // spin
            } else {
                testServo.setPosition(0.5); // stop
            }

            // Show info on Driver Hub screen
            telemetry.addData("Forward/Back", axial);
            telemetry.addData("Strafe",       lateral);
            telemetry.addData("Rotate",       yaw);
            telemetry.update();
        }
    }
}
