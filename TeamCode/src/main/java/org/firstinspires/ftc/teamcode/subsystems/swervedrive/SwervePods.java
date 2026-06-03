package org.firstinspires.ftc.teamcode.subsystems.swervedrive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pathplanner.Pose;

public class SwervePods {

    private final DcMotorEx driveMotor;
    private final CRServo turnMotor;
    private final AnalogInput analogEncoder;
    private double minVoltage = 0.0;
    private double maxVoltage = 3.3;
    private double angleOffset = 0.0;
    private double podPoseX = 0.0;
    private double podPoseY = 0.0;

    private SwervePods(HardwareMap hwm, String motorName, String servoName, String encoderName,
                       double angleOffset, double minVoltage, double maxVoltage) {
        //TODO add the pod construction
        driveMotor = hwm.get(DcMotorEx.class, motorName);
        turnMotor = hwm.get(CRServo.class, servoName);
        analogEncoder = hwm.get(AnalogInput.class, encoderName);

        this.minVoltage = minVoltage;
        this.maxVoltage = maxVoltage;
        this.angleOffset = angleOffset;
    }

    public void setPodPose(Pose pose) {
        podPoseX = pose.getX();
        podPoseY = pose.getY();
    }
}
