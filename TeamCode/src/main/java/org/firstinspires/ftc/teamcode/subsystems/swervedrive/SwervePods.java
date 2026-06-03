package org.firstinspires.ftc.teamcode.subsystems.swervedrive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pathplanner.Pose;

public class SwervePods {
    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double DEFAULT_TURN_KP = 1.8;
    private static final double DEFAULT_TURN_TOLERANCE_RADIANS = Math.toRadians(2.0);

    private final DcMotorEx driveMotor;
    private final CRServo turnMotor;
    private final AnalogInput analogEncoder;
    private double minVoltage;
    private double maxVoltage;
    private double angleOffset = 0.0;
    private double podPoseX = 0.0;
    private double podPoseY = 0.0;
    private double turnKp = DEFAULT_TURN_KP;
    private double turnToleranceRadians = DEFAULT_TURN_TOLERANCE_RADIANS;
    private double lastTargetAngle = 0.0;
    private double lastDrivePower = 0.0;

    public SwervePods(HardwareMap hwm, String motorName, String servoName, String encoderName,
                       double angleOffset, double minVoltage, double maxVoltage) {
        driveMotor = hwm.get(DcMotorEx.class, motorName);
        turnMotor = hwm.get(CRServo.class, servoName);
        analogEncoder = hwm.get(AnalogInput.class, encoderName);

        this.minVoltage = minVoltage;
        this.maxVoltage = maxVoltage;
        this.angleOffset = angleOffset;

        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPodPose(Pose pose) {
        podPoseX = pose.getX();
        podPoseY = pose.getY();
    }

    public Pose getPodPose() {
        return new Pose(podPoseX, podPoseY, 0.0);
    }

    public double getPodPoseX() {
        return podPoseX;
    }

    public double getPodPoseY() {
        return podPoseY;
    }

    public void setAngleOffset(double angleOffset) {
        this.angleOffset = angleOffset;
    }

    public void setTurnKp(double turnKp) {
        this.turnKp = turnKp;
    }

    public void setTurnToleranceRadians(double turnToleranceRadians) {
        this.turnToleranceRadians = Math.abs(turnToleranceRadians);
    }

    public void setDriveReversed(boolean reversed) {
        driveMotor.setDirection(reversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }

    public void setTurnReversed(boolean reversed) {
        turnMotor.setDirection(reversed ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
    }

    public double getCurrentAngle() {
        double voltageRange = maxVoltage - minVoltage;
        if (voltageRange <= 0.0) {
            return 0.0;
        }

        double normalizedVoltage = Range.clip(
                (analogEncoder.getVoltage() - minVoltage) / voltageRange,
                0.0,
                1.0
        );

        return Pose.normalizeRadians((normalizedVoltage * TWO_PI) - angleOffset);
    }

    public double getLastTargetAngle() {
        return lastTargetAngle;
    }

    public double getLastDrivePower() {
        return lastDrivePower;
    }

    public void setTargetState(double drivePower, double targetAngle) {
        double currentAngle = getCurrentAngle();
        double optimizedDrivePower = drivePower;
        double optimizedTargetAngle = targetAngle;
        double angleError = Pose.normalizeRadians(optimizedTargetAngle - currentAngle);

        if (Math.abs(angleError) > Math.PI / 2.0) {
            optimizedDrivePower = -optimizedDrivePower;
            optimizedTargetAngle = Pose.normalizeRadians(optimizedTargetAngle + Math.PI);
            angleError = Pose.normalizeRadians(optimizedTargetAngle - currentAngle);
        }

        double turnPower = Math.abs(angleError) <= turnToleranceRadians
                ? 0.0
                : Range.clip(angleError * turnKp, -1.0, 1.0);

        lastTargetAngle = optimizedTargetAngle;
        lastDrivePower = Range.clip(optimizedDrivePower, -1.0, 1.0);

        turnMotor.setPower(turnPower);
        driveMotor.setPower(lastDrivePower);
    }

    public void stop() {
        driveMotor.setPower(0.0);
        turnMotor.setPower(0.0);
        lastDrivePower = 0.0;
    }

}
