package frc.robot.utils.debugging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;


// UN COMMENT DRIVEBASE RELATED COMMENTS, SUCH AS ON LINES 29, 64-70 AND THE getWheelPositions() FUNCTION
public class WheelRadiusCharacterization extends Command {
    private static final LoggedTunableNumber characterizationSpeed =
        new LoggedTunableNumber("WheelRadiusCharacterization/SpeedRadsPerSec", 0.1);
    private double driveRadius;

    @RequiredArgsConstructor
    public enum Direction {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1);

        private final int value;
    }

    // private final Drive drive;
    private final Direction omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

    private double[] startWheelPositions;

    private double currentEffectiveWheelRadius = 0.0;

    private DoubleSupplier gyroYawRadsSupplier;

    public WheelRadiusCharacterization(
        // Drive drive, 
        Direction omegaDirection, double driveRadiusM) {
        // this.drive = drive;
        this.omegaDirection = omegaDirection;
        driveRadius = driveRadiusM;
        // gyroYawRadsSupplier = () -> drive.getRotation().getRadians();
        // addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Reset
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        accumGyroYawRads = 0.0;

        startWheelPositions = getWheelPositions();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {
        // Run drive at velocity
        // drive.runSwerve(
        //     new ChassisSpeeds(
        //         0.0,
        //         0.0,
        //         omegaDirection.value * (characterizationSpeed.get() * DriveConstants.kMaxAngularSpeedRPS)));

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() -
            lastGyroYawRads);
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        double averageWheelPosition = 0.0;
        double[] wheelPositiions = getWheelPositions();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
        Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
        Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
        Logger.recordOutput(
        "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
            Units.metersToInches(currentEffectiveWheelRadius));
  }

    @Override
    public void end(boolean interrupted) {
        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
        } else {
            Logger.recordOutput(
                "Effective Wheel Radius: ",
                    currentEffectiveWheelRadius);
        }
    }

    public double[] getWheelPositions() {
        // SwerveModulePosition[] positions = drive.getModulePositions();
        double[] doublePosition = new double[4];

        // for (int i = 0; i < 4; i++) {
        //     startWheelPositions[i] = positions[i].angle.getRadians();
        // }

        return doublePosition;
    }
}
