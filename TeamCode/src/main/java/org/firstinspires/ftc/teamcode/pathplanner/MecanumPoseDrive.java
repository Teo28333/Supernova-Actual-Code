package org.firstinspires.ftc.teamcode.pathplanner;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumPoseDrive {
    private static final String FRONT_LEFT_NAME = "front_left_drive";
    private static final String FRONT_RIGHT_NAME = "front_right_drive";
    private static final String BACK_LEFT_NAME = "back_left_drive";
    private static final String BACK_RIGHT_NAME = "back_right_drive";
    private static final String IMU_NAME = "imu";

    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.77953;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private static final double XY_KP = 0.04;
    private static final double HEADING_KP = 0.9;
    private static final double MAX_DRIVE_POWER = 0.6;
    private static final double MAX_TURN_POWER = 0.45;
    private static final double POSITION_TOLERANCE_INCHES = 1.0;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(3.0);

    private final LinearOpMode opMode;
    private final Pose poseEstimate = new Pose();

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private IMU imu;

    private int lastFrontLeftPosition;
    private int lastFrontRightPosition;
    private int lastBackLeftPosition;
    private int lastBackRightPosition;

    public MecanumPoseDrive(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        resetDriveEncoders();

        imu = opMode.hardwareMap.get(IMU.class, IMU_NAME);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void setPoseEstimate(Pose pose) {
        poseEstimate.set(pose);
        rememberEncoderPositions();
    }

    public Pose getPoseEstimate() {
        return poseEstimate.copy();
    }

    public void goToPose(Pose targetPose, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();

        while (opMode.opModeIsActive() && timer.seconds() < timeoutSeconds) {
            updatePoseEstimate();

            double xError = targetPose.getX() - poseEstimate.getX();
            double yError = targetPose.getY() - poseEstimate.getY();
            double headingError = poseEstimate.headingErrorTo(targetPose);

            if (poseEstimate.distanceTo(targetPose) <= POSITION_TOLERANCE_INCHES
                    && Math.abs(headingError) <= HEADING_TOLERANCE_RADIANS) {
                break;
            }

            double xPower = Range.clip(xError * XY_KP, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
            double yPower = Range.clip(yError * XY_KP, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
            double turnPower = Range.clip(headingError * HEADING_KP, -MAX_TURN_POWER, MAX_TURN_POWER);

            driveFieldRelative(xPower, yPower, turnPower);

            opMode.telemetry.addData("Target", "%.1f, %.1f, %.1f deg",
                    targetPose.getX(), targetPose.getY(), targetPose.getHeadingDegrees());
            opMode.telemetry.addData("Pose", "%.1f, %.1f, %.1f deg",
                    poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeadingDegrees());
            opMode.telemetry.addData("Error", "%.1f, %.1f, %.1f deg",
                    xError, yError, Math.toDegrees(headingError));
            opMode.telemetry.update();
        }

        stop();
    }

    public void updatePoseEstimate() {
        int frontLeftPosition = frontLeftDrive.getCurrentPosition();
        int frontRightPosition = frontRightDrive.getCurrentPosition();
        int backLeftPosition = backLeftDrive.getCurrentPosition();
        int backRightPosition = backRightDrive.getCurrentPosition();

        double frontLeftInches = (frontLeftPosition - lastFrontLeftPosition) / COUNTS_PER_INCH;
        double frontRightInches = (frontRightPosition - lastFrontRightPosition) / COUNTS_PER_INCH;
        double backLeftInches = (backLeftPosition - lastBackLeftPosition) / COUNTS_PER_INCH;
        double backRightInches = (backRightPosition - lastBackRightPosition) / COUNTS_PER_INCH;

        double robotForward = (frontLeftInches + frontRightInches + backLeftInches + backRightInches) / 4.0;
        double robotRight = (frontLeftInches - frontRightInches - backLeftInches + backRightInches) / 4.0;
        double heading = getHeading();

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double fieldX = (robotRight * cos) - (robotForward * sin);
        double fieldY = (robotRight * sin) + (robotForward * cos);

        poseEstimate.set(
                poseEstimate.getX() + fieldX,
                poseEstimate.getY() + fieldY,
                heading
        );

        lastFrontLeftPosition = frontLeftPosition;
        lastFrontRightPosition = frontRightPosition;
        lastBackLeftPosition = backLeftPosition;
        lastBackRightPosition = backRightPosition;
    }

    public void driveFieldRelative(double xPower, double yPower, double turnPower) {
        double heading = poseEstimate.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double robotRight = (xPower * cos) + (yPower * sin);
        double robotForward = (-xPower * sin) + (yPower * cos);

        driveRobotRelative(robotForward, robotRight, turnPower);
    }

    public void driveRobotRelative(double forwardPower, double rightPower, double turnPower) {
        double frontLeftPower = forwardPower + rightPower + turnPower;
        double frontRightPower = forwardPower - rightPower - turnPower;
        double backLeftPower = forwardPower - rightPower + turnPower;
        double backRightPower = forwardPower + rightPower - turnPower;

        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        ));

        frontLeftDrive.setPower(frontLeftPower / maxPower);
        frontRightDrive.setPower(frontRightPower / maxPower);
        backLeftDrive.setPower(backLeftPower / maxPower);
        backRightDrive.setPower(backRightPower / maxPower);
    }

    public void stop() {
        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
    }

    private void resetDriveEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rememberEncoderPositions();
    }

    private void rememberEncoderPositions() {
        lastFrontLeftPosition = frontLeftDrive.getCurrentPosition();
        lastFrontRightPosition = frontRightDrive.getCurrentPosition();
        lastBackLeftPosition = backLeftDrive.getCurrentPosition();
        lastBackRightPosition = backRightDrive.getCurrentPosition();
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
