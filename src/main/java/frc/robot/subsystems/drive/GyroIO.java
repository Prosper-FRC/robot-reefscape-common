package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public Rotation2d yawVelocityPS = Rotation2d.fromDegrees(0.0);
    }

    public default void updateInputs(GyroInputs inputs) {}

    public default void resetGyro(Rotation2d rotation) {}
}
