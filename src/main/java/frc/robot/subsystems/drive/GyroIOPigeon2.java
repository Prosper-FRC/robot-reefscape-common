package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOPigeon2 implements GyroIO {
    private Pigeon2 gyro = new Pigeon2(10, "drivebase");

    private StatusSignal<Angle> yaw = gyro.getYaw();
    private StatusSignal<AngularVelocity> yawVelocity = gyro.getAngularVelocityXWorld();

    public GyroIOPigeon2() {
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.getConfigurator().setYaw(0.0);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, yaw, yawVelocity);

        gyro.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityPS = Rotation2d.fromDegrees(yawVelocity.getValueAsDouble());
    }

    @Override
    public void resetGyro(Rotation2d rotation) {
        gyro.setYaw(rotation.getDegrees());
    }
}
