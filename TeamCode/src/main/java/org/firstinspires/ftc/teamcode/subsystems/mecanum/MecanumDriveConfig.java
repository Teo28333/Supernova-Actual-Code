package org.firstinspires.ftc.teamcode.subsystems.mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumDriveConfig {
    public String frontLeftMotorName = "frontLeft";
    public String frontRightMotorName = "frontRight";
    public String backLeftMotorName = "backLeft";
    public String backRightMotorName = "backRight";

    public DcMotorSimple.Direction frontLeftDirection = DcMotorSimple.Direction.FORWARD;
    public DcMotorSimple.Direction frontRightDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorSimple.Direction backLeftDirection = DcMotorSimple.Direction.FORWARD;
    public DcMotorSimple.Direction backRightDirection = DcMotorSimple.Direction.REVERSE;

    public DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
    public DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public double maxPower = 1.0;
    public boolean predictiveBrakingEnabled = true;
    public double maxLinearDeceleration = 40.0;
    public double maxAngularDeceleration = Math.toRadians(360.0);
    public double brakingDistanceBuffer = 2.0;
    public double minimumBrakeScale = 0.12;
    public boolean xLockEnabled = false;
    public double xLockDeadband = 0.03;

    public MecanumDriveConfig withMotorNames(String frontLeftMotorName, String frontRightMotorName,
                                             String backLeftMotorName, String backRightMotorName) {
        this.frontLeftMotorName = frontLeftMotorName;
        this.frontRightMotorName = frontRightMotorName;
        this.backLeftMotorName = backLeftMotorName;
        this.backRightMotorName = backRightMotorName;
        return this;
    }

    public MecanumDriveConfig withMotorDirections(DcMotorSimple.Direction frontLeftDirection,
                                                  DcMotorSimple.Direction frontRightDirection,
                                                  DcMotorSimple.Direction backLeftDirection,
                                                  DcMotorSimple.Direction backRightDirection) {
        this.frontLeftDirection = frontLeftDirection;
        this.frontRightDirection = frontRightDirection;
        this.backLeftDirection = backLeftDirection;
        this.backRightDirection = backRightDirection;
        return this;
    }

    public MecanumDriveConfig withZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
        return this;
    }

    public MecanumDriveConfig withBrakeMode(boolean brakeModeEnabled) {
        zeroPowerBehavior = brakeModeEnabled ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        return this;
    }

    public MecanumDriveConfig withMaxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public MecanumDriveConfig withPredictiveBraking(boolean enabled) {
        predictiveBrakingEnabled = enabled;
        return this;
    }

    public MecanumDriveConfig withMaxLinearDeceleration(double maxLinearDeceleration) {
        this.maxLinearDeceleration = maxLinearDeceleration;
        return this;
    }

    public MecanumDriveConfig withMaxAngularDeceleration(double maxAngularDeceleration) {
        this.maxAngularDeceleration = maxAngularDeceleration;
        return this;
    }

    public MecanumDriveConfig withBrakingDistanceBuffer(double brakingDistanceBuffer) {
        this.brakingDistanceBuffer = brakingDistanceBuffer;
        return this;
    }

    public MecanumDriveConfig withMinimumBrakeScale(double minimumBrakeScale) {
        this.minimumBrakeScale = minimumBrakeScale;
        return this;
    }

    public MecanumDriveConfig withXLock(boolean enabled) {
        xLockEnabled = enabled;
        return this;
    }

    public MecanumDriveConfig withXLockDeadband(double xLockDeadband) {
        this.xLockDeadband = xLockDeadband;
        return this;
    }
}
