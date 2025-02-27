package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleInputs {
        public boolean isDriveConnected = true;
        public double drivePositionM = 0.0;
        public double driveVelocityMPS = 0.0;
        public double driveStatorCurrentAmps = 0.0;
        public double driveSupplyCurrentAmps = 0.0;
        public double driveTorqueCurrentAmps = 0.0;
        public double driveTemperatureCelsius = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveMotorVolts = 0.0;
        public double driveAccelerationMPSS = 0.0;

        public boolean isAzimuthConnected = true;
        public Rotation2d azimuthPosition = new Rotation2d();
        public Rotation2d azimuthVelocity = new Rotation2d();
        public double azimuthStatorCurrentAmps = 0.0;
        public double azimuthSupplyCurrentAmps = 0.0;
        // public double azimuthTorqueCurrentAmps = 0.0;
        public double azimuthTemperatureCelsius = 0.0;
        public double azimuthAppliedVolts = 0.0;
        public double azimuthMotorVolts = 0.0;

        public boolean isCancoderConnected = true;
        public Rotation2d azimuthAbsolutePosition = new Rotation2d();
    }

    public default void updateInputs(ModuleInputs inputs) {}

    public default void setDriveVelocity(double velocityMPS, double feedforward) {}

    public default void setDriveVolts(double volts) {}

    public default void setDriveAmperage(double amps) {}

    public default void setDrivePID(double kP, double kI, double kD) {}

    public default void setAzimuthVolts(double votls) {}

    public default void setAzimuthPosition(Rotation2d rotation, double feedforward) {}

    public default void resetAzimuthEncoder() {}

    public default void setAzimuthPID(double kP, double kI, double kD) {}
}
