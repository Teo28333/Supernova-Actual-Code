package org.firstinspires.ftc.teamcode.pathplanner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Pose Auto Example", group = "Path Planner")
public class PoseAutoExample extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumPoseDrive drive = new MecanumPoseDrive(this);
        drive.init();

        drive.setPoseEstimate(Pose.zero());

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        drive.goToPose(Pose.fromDegrees(24.0, 0.0, 0.0), 4.0);
        drive.goToPose(Pose.fromDegrees(24.0, 24.0, 90.0), 5.0);
        drive.goToPose(Pose.fromDegrees(0.0, 24.0, 180.0), 5.0);

        drive.stop();
    }
}
