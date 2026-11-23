package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Main Field-Centric TeleOp OpMode extending TeleOp.
 * Inherits all operator controls, subsystems, and telemetry from TeleOp,
 * and overrides drive execution for Field-Centric driving.
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Field Centric", group = "Iterative Opmode")
public class TelopFieldCentric extends TeleOp {

    @Override
    protected void calculateAndExecuteDrive() {
        // Reset Field-Centric Heading using Back Button
        if (driver.wasJustPressed(GamepadController.Button.OPTIONS)) {
//            robot.resetHeading();
            robot.imu.resetYaw();   //from Claude
        }

        // 1. Read Joystick Inputs with Exponential Response Curve & Deadzone
        double driveInput  = -driver.getLeftStickYCurve(STICK_CURVE_EXPONENT);
        double strafeInput = -driver.getLeftStickXCurve(STICK_CURVE_EXPONENT);
        double yaw    = -driver.getRightStickXCurve(STICK_CURVE_EXPONENT) * TURN_SPEED;

        // --- Field-Centric Transformation ---
        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);  //from Claude

        double rotX = strafeInput * Math.cos(-botHeading) - driveInput * Math.sin(-botHeading);
        double rotY = strafeInput * Math.sin(-botHeading) + driveInput * Math.cos(-botHeading);

        rotX *= 1.1; // counteract imperfect strafing

        // 2. Precise Micro-Stepping using D-Pad (overrides joystick input when pressed)
        if (driver.isDown(GamepadController.Button.DPAD_UP)) {
            rotY = SLOW_DRIVE_SPEED / 2.0;
            rotX = 0;
        } else if (driver.isDown(GamepadController.Button.DPAD_DOWN)) {
            rotY = -SLOW_DRIVE_SPEED / 2.0;
            rotX = 0;
        } else if (driver.isDown(GamepadController.Button.DPAD_LEFT)) {
            rotX = SLOW_DRIVE_SPEED / 2.0;
            rotY = 0;
        } else if (driver.isDown(GamepadController.Button.DPAD_RIGHT)) {
            rotX = -SLOW_DRIVE_SPEED / 2.0;
            rotY = 0;
        }

        // Send movement command to hardware (rotY = drive, rotX = strafe, yaw = turn)
        robot.moveRobot(rotY, rotX, yaw);
    }
}
