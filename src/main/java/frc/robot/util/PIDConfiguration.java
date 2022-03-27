package frc.robot.util;

public class PIDConfiguration {
    private final double kP, kI, kD;

    public PIDConfiguration(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }

    public static PIDConfiguration of(double kP, double kI, double kD) {
        return new PIDConfiguration(kP, kI, kD);
    }
}
