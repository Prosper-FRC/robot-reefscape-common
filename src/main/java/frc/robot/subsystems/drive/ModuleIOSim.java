package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;


public class ModuleIOSim implements ModuleIO {

    private DCMotorSim driveMotor = 
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.004, kDriveGearRatio), 
        DCMotor.getKrakenX60(1), 0.0, 0.0);

    private DCMotorSim azimuthMotor = 
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.025, kAzimuthGearRatio), 
        DCMotor.getKrakenX60(1), 0.0, 0.0);


    private double driveAppliedVolts = 0.0;
    private double azimuthAppliedVolts = 0.0;

    private PIDController drivePID = kModuleControllerConfigs.driveController();
    private SimpleMotorFeedforward driveFF = kModuleControllerConfigs.driveFF();

    private PIDController azimuthPID = kModuleControllerConfigs.azimuthController();
    private SimpleMotorFeedforward azimuthFF = kModuleControllerConfigs.azimuthFF();

    public ModuleIOSim() {
        azimuthPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveMotor.update(0.02);
        azimuthMotor.update(0.02);

        inputs.drivePositionM = driveMotor.getAngularPositionRotations() * DriveConstants.kWheelCircumferenceMeters;
        inputs.driveVelocityMPS = (driveMotor.getAngularVelocityRPM() * DriveConstants.kWheelCircumferenceMeters) / 60.0;
        inputs.driveTempC = new double[]{0.0};
        inputs.driveConnected = true;
        inputs.driveMotorVolts = driveAppliedVolts;
        inputs.driveStatorAmps = new double[] {Math.abs(driveMotor.getCurrentDrawAmps())};

        inputs.azimuthAbsolutePosition = new Rotation2d(azimuthMotor.getAngularPositionRad());
        inputs.azimuthPosition = new Rotation2d(azimuthMotor.getAngularPositionRad());
        inputs.azimuthStatorAmps = new double[] {Math.abs(azimuthMotor.getCurrentDrawAmps())};
        inputs.azimuthTempC = new double[] {0.0};
        inputs.azimuthMotorVolts = azimuthAppliedVolts;
        inputs.azimuthConnected = true;
    }

    /////////// DRIVE MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setDriveVolts(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveMotor.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setDriveVelocity(double velocityMPS) {
        setDriveVolts(
            drivePID.calculate(
                driveMotor.getAngularVelocityRPM() * DriveConstants.kWheelCircumferenceMeters / 60, 
                velocityMPS) + driveFF.calculate(velocityMPS));
    }

    @Override
    public void setDriveGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        drivePID.setPID(kP, kI, kD);
        driveFF = new SimpleMotorFeedforward(kS, kV, kA);
    }


    /////////// AZIMUTH MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setAzimuthVolts(double volts) {
        azimuthAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        azimuthMotor.setInputVoltage(azimuthAppliedVolts);
    }

    @Override
    public void setAzimuthPosition(Rotation2d position) {
        setAzimuthVolts(azimuthPID.calculate(azimuthMotor.getAngularPositionRad(), position.getRadians()));
    }

    @Override
    public void setAzimuthGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        azimuthPID.setPID(kP, kI, kD);
        azimuthFF = new SimpleMotorFeedforward(kS, kV, kA);
    }
}