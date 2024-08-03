package org.firstinspires.ftc.teamcode.opmode;

public class CustomPID {
    private double kP, kI, kD;
    private double setpoint;
    private double integralSum;
    private double previousError;
    private double previousTime;

    public CustomPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.setpoint = 0;
        this.integralSum = 0;
        this.previousError = 0;
        this.previousTime = System.nanoTime() / 1e9; // time in seconds
    }

    public void setTargetPosition(double setpoint) {
        this.setpoint = setpoint;
        this.integralSum = 0;
        this.previousError = 0;
    }

    public double update(double currentPosition) {
        double currentTime = System.nanoTime() / 1e9; // time in seconds
        double deltaTime = currentTime - previousTime;
        previousTime = currentTime;

        double error = setpoint - currentPosition;
        integralSum += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;
        previousError = error;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);
        return output;
    }

    public void setTunings(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // Getters for PID coefficients, useful for debugging or tuning purposes
    public double getKP() {
        return kP;
    }

    public double getKI() {
        return kI;
    }

    public double getKD() {
        return kD;
    }
}
