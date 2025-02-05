package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class SwerveModuleInputs {
        public boolean driveConnected = true;
        public double drivePositionM = 0.0;
        public double driveVelocityMPS = 0.0;
        public double[] driveStatorAmps = new double[] {0.0};
        public double[] driveTempC = new double[] {0.0};
        public double driveMotorVolts = 0.0;

        public boolean azimuthConnected = true;
        public Rotation2d azimuthPosition = new Rotation2d();
        public Rotation2d azimuthAbsolutePosition = new Rotation2d();
        public double[] azimuthStatorAmps = new double[] {0.0};
        public double[] azimuthTempC = new double[] {0.0};
        public double azimuthMotorVolts = 0.0;
    }
    
    public default void updateInputs(SwerveModuleInputs inputs) {}

    public default void setDriveVelocity(double velocityMPS) {}

    public default void setDriveVolts(double volts) {}

    public default void setDriveGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

    public default void setAzimuthVolts(double votls) {}

    public default void setAzimuthGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

    public default void setAzimuthPosition(Rotation2d rotation) {}

    public default void resetAzimuthEncoder() {}


}