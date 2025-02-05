package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroInputs{
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
    }

    public default void updateInputs(GyroInputs inputs) {}

    public default void resetGyro(Rotation2d rotation) {}
}
