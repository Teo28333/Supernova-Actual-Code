package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Follower;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;
import org.firstinspires.ftc.teamcode.config.SwerveAutonTuningConfig;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.localization.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.localization.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.swervedrive.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.swervedrive.SwervePods;

@TeleOp(name = "Tuning")
public class Tuning extends OpMode {
    private Localizer localizer;
    private Drivetrain drivetrain;
    private SwerveDrive swerveDrive;
    private Follower follower;
    private boolean initialized = false;
    private boolean holdCommanded = false;
    private String followerSignature = "";
    private String holdTargetSignature = "";
    private boolean previousDpadUp;
    private boolean previousDpadDown;
    private boolean previousDpadLeft;
    private boolean previousDpadRight;
    private boolean previousLeftBumper;
    private boolean previousRightBumper;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        updateInitSelection();
        telemetry.addData("opmode", "Tuning");
        telemetry.addData("mode", SwerveAutonTuningConfig.MODE);
        telemetry.addData("drivetrain", SwerveAutonTuningConfig.DRIVETRAIN);
        telemetry.addData("localizer", SwerveAutonTuningConfig.LOCALIZER);
        telemetry.addLine("dpad up/down: mode");
        telemetry.addLine("dpad left/right: localizer");
        telemetry.addLine("bumpers: drivetrain");
    }

    @Override
    public void start() {
        buildSelectedRobot();
    }

    @Override
    public void loop() {
        if (!initialized) {
            buildSelectedRobot();
        }

        applyLiveConfig();
        localizer.update();
        Pose pose = localizer.getPoseEstimate();

        switch (SwerveAutonTuningConfig.MODE) {
            case POD_ANGLE:
                tunePodAngle();
                holdCommanded = false;
                break;
            case X_LOCK:
                xLock();
                holdCommanded = false;
                break;
            case AUTON_HOLD:
                followerHold();
                break;
            case JOYSTICK:
            default:
                joystickDrive(pose);
                holdCommanded = false;
                break;
        }

        if (gamepad1.options || gamepad1.start) {
            setStartingPose();
            holdCommanded = false;
        }

        addTelemetry(pose);
    }

    @Override
    public void stop() {
        if (drivetrain != null) {
            drivetrain.stop();
        }
    }

    private void buildSelectedRobot() {
        localizer = createLocalizer();
        setStartingPose();

        switch (SwerveAutonTuningConfig.DRIVETRAIN) {
            case MECANUM:
                drivetrain = new MecanumDrive(hardwareMap, SwerveAutonTuningConfig.mecanumDrive());
                swerveDrive = null;
                break;
            case SWERVE:
            default:
                swerveDrive = new SwerveDrive(hardwareMap, localizer, SwerveAutonTuningConfig.swerveDrive());
                drivetrain = swerveDrive;
                break;
        }

        follower = new Follower(drivetrain, localizer, SwerveAutonTuningConfig.follower());
        follower.setStartingPose(SwerveAutonTuningConfig.startingPose());
        followerSignature = followerSignature();
        initialized = true;
    }

    private Localizer createLocalizer() {
        switch (SwerveAutonTuningConfig.LOCALIZER) {
            case THREE_WHEEL:
                return new ThreeWheelLocalizer(hardwareMap, SwerveAutonTuningConfig.threeWheel());
            case OTOS:
                OTOSLocalizer otos = new OTOSLocalizer(hardwareMap, SwerveAutonTuningConfig.OTOS_NAME);
                otos.configure(SwerveAutonTuningConfig.otos());
                return otos;
            case PINPOINT:
            default:
                PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, SwerveAutonTuningConfig.PINPOINT_NAME);
                pinpoint.configure(SwerveAutonTuningConfig.pinpoint());
                return pinpoint;
        }
    }

    private void applyLiveConfig() {
        if (swerveDrive != null) {
            swerveDrive.setConfig(SwerveAutonTuningConfig.swerveDrive());
            swerveDrive.setAllTurnPidf(
                    SwerveAutonTuningConfig.POD_KP,
                    SwerveAutonTuningConfig.POD_KI,
                    SwerveAutonTuningConfig.POD_KD,
                    SwerveAutonTuningConfig.POD_KF
            );
            swerveDrive.setAllTurnToleranceRadians(Math.toRadians(SwerveAutonTuningConfig.POD_TOLERANCE_DEGREES));
            swerveDrive.setAllTurnIntegralLimit(SwerveAutonTuningConfig.POD_INTEGRAL_LIMIT);
        } else if (drivetrain instanceof MecanumDrive) {
            ((MecanumDrive) drivetrain).setConfig(SwerveAutonTuningConfig.mecanumDrive());
        }

        String newSignature = followerSignature();
        if (!newSignature.equals(followerSignature)) {
            follower.setConfig(SwerveAutonTuningConfig.follower());
            followerSignature = newSignature;
            holdCommanded = false;
        }
    }

    private void joystickDrive(Pose pose) {
        double scale = Range.clip(SwerveAutonTuningConfig.DRIVE_SCALE, 0.0, 1.0);
        Pose command = new Pose(
                applyDeadband(gamepad1.left_stick_x, 0.05) * scale,
                applyDeadband(-gamepad1.left_stick_y, 0.05) * scale,
                applyDeadband(gamepad1.right_stick_x, 0.05) * scale
        );
        drivetrain.driveFieldCentric(command, pose, localizer.getVelocity(), pose);
    }

    private void tunePodAngle() {
        if (swerveDrive == null) {
            drivetrain.stop();
            return;
        }

        SwervePods[] pods = swerveDrive.getPods();
        int selectedPod = SwerveAutonTuningConfig.SELECTED_POD;
        double targetAngle = Math.toRadians(SwerveAutonTuningConfig.TARGET_POD_ANGLE_DEGREES);

        if (selectedPod >= 0 && selectedPod < pods.length) {
            pods[selectedPod].pointAt(targetAngle);
            return;
        }

        for (SwervePods pod : pods) {
            pod.pointAt(targetAngle);
        }
    }

    private void xLock() {
        if (swerveDrive != null && SwerveAutonTuningConfig.X_LOCK_ENABLED) {
            swerveDrive.xLock();
        } else {
            drivetrain.stop();
        }
    }

    private void followerHold() {
        Pose target = new Pose(
                SwerveAutonTuningConfig.TARGET_X,
                SwerveAutonTuningConfig.TARGET_Y,
                Math.toRadians(SwerveAutonTuningConfig.TARGET_HEADING_DEGREES)
        );

        if (!holdCommanded) {
            follower.holdPoint(target);
            holdTargetSignature = holdTargetSignature();
            holdCommanded = true;
        } else if (!holdTargetSignature().equals(holdTargetSignature)) {
            follower.holdPoint(target);
            holdTargetSignature = holdTargetSignature();
        }

        follower.update();
    }

    private void setStartingPose() {
        localizer.setStartingPose(SwerveAutonTuningConfig.startingPose());
        if (follower != null) {
            follower.setStartingPose(SwerveAutonTuningConfig.startingPose());
        }
    }

    private void addTelemetry(Pose pose) {
        telemetry.addData("mode", SwerveAutonTuningConfig.MODE);
        telemetry.addData("drivetrain", SwerveAutonTuningConfig.DRIVETRAIN);
        telemetry.addData("localizer", SwerveAutonTuningConfig.LOCALIZER);
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("heading", pose.getHeadingDegrees());
        telemetry.addData("velocity x", localizer.getVelocity().getX());
        telemetry.addData("velocity y", localizer.getVelocity().getY());
        telemetry.addData("velocity heading", Math.toDegrees(localizer.getVelocity().getHeading()));

        if (localizer instanceof PinpointLocalizer) {
            telemetry.addData("pinpoint", ((PinpointLocalizer) localizer).getDeviceStatus());
        }

        if (swerveDrive != null) {
            SwervePods[] pods = swerveDrive.getPods();
            for (int i = 0; i < pods.length; i++) {
                telemetry.addData("pod " + i + " angle deg", Math.toDegrees(pods[i].getCurrentAngle()));
                telemetry.addData("pod " + i + " target deg", Math.toDegrees(pods[i].getLastTargetAngle()));
                telemetry.addData("pod " + i + " drive", pods[i].getLastDrivePower());
                telemetry.addData("pod " + i + " turn", pods[i].getLastTurnPower());
                telemetry.addData("pod " + i + " error deg", Math.toDegrees(pods[i].getLastAngleError()));
            }
        }
    }

    private void updateInitSelection() {
        if (pressed(gamepad1.dpad_up, previousDpadUp)) {
            SwerveAutonTuningConfig.MODE = previousEnum(
                    SwerveAutonTuningConfig.MODE,
                    SwerveAutonTuningConfig.TuningMode.values()
            );
        }
        if (pressed(gamepad1.dpad_down, previousDpadDown)) {
            SwerveAutonTuningConfig.MODE = nextEnum(
                    SwerveAutonTuningConfig.MODE,
                    SwerveAutonTuningConfig.TuningMode.values()
            );
        }
        if (pressed(gamepad1.dpad_left, previousDpadLeft)) {
            SwerveAutonTuningConfig.LOCALIZER = previousEnum(
                    SwerveAutonTuningConfig.LOCALIZER,
                    SwerveAutonTuningConfig.LocalizerType.values()
            );
        }
        if (pressed(gamepad1.dpad_right, previousDpadRight)) {
            SwerveAutonTuningConfig.LOCALIZER = nextEnum(
                    SwerveAutonTuningConfig.LOCALIZER,
                    SwerveAutonTuningConfig.LocalizerType.values()
            );
        }
        if (pressed(gamepad1.left_bumper, previousLeftBumper)) {
            SwerveAutonTuningConfig.DRIVETRAIN = previousEnum(
                    SwerveAutonTuningConfig.DRIVETRAIN,
                    SwerveAutonTuningConfig.DrivetrainType.values()
            );
        }
        if (pressed(gamepad1.right_bumper, previousRightBumper)) {
            SwerveAutonTuningConfig.DRIVETRAIN = nextEnum(
                    SwerveAutonTuningConfig.DRIVETRAIN,
                    SwerveAutonTuningConfig.DrivetrainType.values()
            );
        }

        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;
        previousDpadLeft = gamepad1.dpad_left;
        previousDpadRight = gamepad1.dpad_right;
        previousLeftBumper = gamepad1.left_bumper;
        previousRightBumper = gamepad1.right_bumper;
    }

    private static boolean pressed(boolean current, boolean previous) {
        return current && !previous;
    }

    private static <T extends Enum<T>> T nextEnum(T value, T[] values) {
        return values[(value.ordinal() + 1) % values.length];
    }

    private static <T extends Enum<T>> T previousEnum(T value, T[] values) {
        return values[(value.ordinal() + values.length - 1) % values.length];
    }

    private static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }

        return Range.clip(value, -1.0, 1.0);
    }

    private static String followerSignature() {
        return SwerveAutonTuningConfig.TRANSLATION_KP + ":"
                + SwerveAutonTuningConfig.TRANSLATION_KI + ":"
                + SwerveAutonTuningConfig.TRANSLATION_KD + ":"
                + SwerveAutonTuningConfig.TRANSLATION_KF + ":"
                + SwerveAutonTuningConfig.HEADING_KP + ":"
                + SwerveAutonTuningConfig.HEADING_KI + ":"
                + SwerveAutonTuningConfig.HEADING_KD + ":"
                + SwerveAutonTuningConfig.HEADING_KF + ":"
                + SwerveAutonTuningConfig.CENTRIPETAL_CORRECTION_ENABLED + ":"
                + SwerveAutonTuningConfig.CENTRIPETAL_CORRECTION_SCALE + ":"
                + SwerveAutonTuningConfig.PREDICTIVE_BRAKING_ENABLED + ":"
                + SwerveAutonTuningConfig.MAX_TRANSLATION_POWER + ":"
                + SwerveAutonTuningConfig.MAX_HEADING_POWER;
    }

    private static String holdTargetSignature() {
        return SwerveAutonTuningConfig.TARGET_X + ":"
                + SwerveAutonTuningConfig.TARGET_Y + ":"
                + SwerveAutonTuningConfig.TARGET_HEADING_DEGREES;
    }
}
