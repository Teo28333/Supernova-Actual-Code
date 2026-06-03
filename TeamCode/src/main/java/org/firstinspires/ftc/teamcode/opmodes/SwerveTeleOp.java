package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;
import org.firstinspires.ftc.teamcode.config.TeleOpSwerveConfig;
import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.swervedrive.SwerveDrive;

@TeleOp(name = "Swerve TeleOp")
@Disabled
public class SwerveTeleOp extends OpMode {
    private PinpointLocalizer localizer;
    private SwerveDrive drive;

    @Override
    public void init() {
        localizer = new PinpointLocalizer(hardwareMap, TeleOpSwerveConfig.PINPOINT_NAME);
        localizer.configure(TeleOpSwerveConfig.pinpoint());
        drive = new SwerveDrive(hardwareMap, localizer, TeleOpSwerveConfig.swerveDrive());
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.driveFieldCentric(
                applyDeadband(forward, 0.05),
                applyDeadband(strafe, 0.05),
                applyDeadband(rotate, 0.05)
        );

        if (gamepad1.options || gamepad1.start) {
            localizer.setHeading(0.0);
        }

        Pose pose = localizer.getPoseEstimate();
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("heading", pose.getHeadingDegrees());
        telemetry.addData("pinpoint", localizer.getDeviceStatus());
    }

    @Override
    public void stop() {
        drive.stop();
    }

    private static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }

        return Range.clip(value, -1.0, 1.0);
    }
}
