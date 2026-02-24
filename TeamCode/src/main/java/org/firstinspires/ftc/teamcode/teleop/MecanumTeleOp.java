package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Servo
    private Servo testServo;

    // Pinpoint odometry computer
    private GoBildaPinpointDriver pinpoint;

    // Heading correction P gain — raise if correction is too slow, lower if it oscillates
    private static final double HEADING_KP = 0.5;

    // Minimum stick input to count as intentional rotation
    private static final double ROTATION_DEADBAND = 0.05;

    @Override
    public void runOpMode() {

        // Connect motors to hardware map
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Connect servo
        testServo = hardwareMap.get(Servo.class, "testServo");

        // Connect Pinpoint and calibrate IMU (robot must be still)
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        // Reverse left side motors so all wheels drive forward together
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Ready to run!");
        telemetry.update();

        waitForStart();

        // Target heading for correction — starts at 0 (wherever robot is facing on init)
        double targetHeading = 0;

        while (opModeIsActive()) {

            // Refresh Pinpoint data
            pinpoint.update();
            double robotHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

            // Read gamepad inputs
            double axial   = -gamepad1.left_stick_y; // Forward / backward
            double lateral =  gamepad1.left_stick_x; // Strafe left / right
            double yaw     =  gamepad1.right_stick_x; // Rotate

            // Field-centric: rotate joystick inputs by -robotHeading
            // This makes "stick forward" always push the robot away from the driver
            double rotAxial   =  axial  * Math.cos(-robotHeading) - lateral * Math.sin(-robotHeading);
            double rotLateral =  axial  * Math.sin(-robotHeading) + lateral * Math.cos(-robotHeading);

            // Heading correction: when driver isn't rotating, hold the last heading
            if (Math.abs(yaw) > ROTATION_DEADBAND) {
                // Driver is actively turning — update target to current heading
                targetHeading = robotHeading;
            } else {
                // Driver released stick — apply P correction toward target heading
                double headingError = targetHeading - robotHeading;

                // Normalize error to [-π, π] to always take the shortest path
                while (headingError >  Math.PI) headingError -= 2 * Math.PI;
                while (headingError < -Math.PI) headingError += 2 * Math.PI;

                yaw = HEADING_KP * headingError;
            }

            // Mecanum wheel math
            double frontLeftPower  = rotAxial + rotLateral + yaw;
            double frontRightPower = rotAxial - rotLateral - yaw;
            double backLeftPower   = rotAxial - rotLateral + yaw;
            double backRightPower  = rotAxial + rotLateral - yaw;

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
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotHeading));
            telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetHeading));
            telemetry.addData("Forward/Back", "%.2f", axial);
            telemetry.addData("Strafe",       "%.2f", lateral);
            telemetry.addData("Rotate",       "%.2f", yaw);
            telemetry.update();
        }
    }
}
