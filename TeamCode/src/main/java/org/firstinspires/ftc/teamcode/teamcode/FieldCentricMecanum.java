package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp(name = "Field Centric Mecanum", group = "Drive")
public class FieldCentricMecanum extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Motor names must match your robot configuration
        DcMotor frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft   = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight  = hardwareMap.dcMotor.get("backRight");

        // Reverse the left side (flip to the right side instead if your robot drives backward)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU setup: CHANGE these to match how your Control Hub is mounted
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;  // forward (stick is inverted)
            double x  = gamepad1.left_stick_x;   // strafe
            double rx = gamepad1.right_stick_x;  // turn

            // Re-zero heading: face the robot away from you, then press Options
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the joystick vector by the negative of the robot heading
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1; // counteract imperfect strafing

            // Scale so no motor power exceeds 1 while keeping ratios
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            frontLeft.setPower((rotY + rotX + rx) / denominator);
            backLeft.setPower((rotY - rotX + rx) / denominator);
            frontRight.setPower((rotY - rotX - rx) / denominator);
            backRight.setPower((rotY + rotX - rx) / denominator);

            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.update();
        }
    }
}
