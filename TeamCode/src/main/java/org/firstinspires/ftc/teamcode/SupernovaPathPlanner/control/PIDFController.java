package org.firstinspires.ftc.teamcode.SupernovaPathPlanner.control;

import com.qualcomm.robotcore.util.Range;

public class PIDFController {
    private PIDFCoefficients coefficients;
    private double target = 0.0;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 1.0;
    private double outputMin = -1.0;
    private double outputMax = 1.0;
    private long lastUpdateTimeNanos = 0L;

    public PIDFController(PIDFCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    public void setCoefficients(PIDFCoefficients coefficients) {
        this.coefficients = coefficients;
        reset();
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setIntegralLimit(double integralLimit) {
        this.integralLimit = Math.abs(integralLimit);
    }

    public void setOutputBounds(double outputMin, double outputMax) {
        this.outputMin = outputMin;
        this.outputMax = outputMax;
    }

    public double update(double current) {
        return updateError(target - current);
    }

    public double updateError(double error) {
        long now = System.nanoTime();
        double dt = lastUpdateTimeNanos == 0L
                ? 0.0
                : (now - lastUpdateTimeNanos) / 1_000_000_000.0;

        if (dt > 0.0) {
            integralSum += error * dt;
            integralSum = Range.clip(integralSum, -integralLimit, integralLimit);
        }

        double derivative = dt > 0.0 ? (error - lastError) / dt : 0.0;
        double feedForward = coefficients.kF == 0.0 ? 0.0 : Math.copySign(coefficients.kF, error);
        double output = (coefficients.kP * error)
                + (coefficients.kI * integralSum)
                + (coefficients.kD * derivative)
                + feedForward;

        lastError = error;
        lastUpdateTimeNanos = now;

        return Range.clip(output, outputMin, outputMax);
    }

    public void reset() {
        integralSum = 0.0;
        lastError = 0.0;
        lastUpdateTimeNanos = 0L;
    }
}
