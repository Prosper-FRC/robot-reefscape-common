package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
    private DCMotorSim driveMotor = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, kDriveGearing), 
            DCMotor.getKrakenX60Foc(1), 0.0, 0.0);
    private DCMotorSim azimuthMotor = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.025, kAzimuthGearing), 
            DCMotor.getKrakenX60Foc(1), 0.0, 0.0);

    private double driveAppliedVolts = 0.0;
    private double azimuthAppliedVolts = 0.0;

    private PIDController drivePID = kModuleControllerConfigs.driveController();

    private PIDController azimuthPID = kModuleControllerConfigs.azimuthController();

    public ModuleIOSim() {
        azimuthPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        driveMotor.update(0.02);
        azimuthMotor.update(0.02);

        inputs.drivePositionM = driveMotor.getAngularPositionRotations() * kCircumferenceMeters;
        inputs.driveVelocityMPS = (driveMotor.getAngularVelocityRPM() * kCircumferenceMeters) / 60.0;
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveStatorCurrentAmps = new double[] {Math.abs(driveMotor.getCurrentDrawAmps())};
        inputs.driveTemperatureCelsius = new double[] {0.0};
        inputs.azimuthAppliedVolts = azimuthAppliedVolts;
        inputs.azimuthMotorVolts = azimuthAppliedVolts;

        inputs.azimuthAbsolutePosition = new Rotation2d(azimuthMotor.getAngularPositionRad());
        inputs.azimuthPosition = new Rotation2d(azimuthMotor.getAngularPositionRad());
        inputs.azimuthVelocity = Rotation2d.fromRadians(azimuthMotor.getAngularVelocityRadPerSec());
        inputs.azimuthStatorCurrentAmps = new double[] {Math.abs(azimuthMotor.getCurrentDrawAmps())};
        inputs.azimuthTemperatureCelsius = new double[] {0.0};
        inputs.azimuthAppliedVolts = azimuthAppliedVolts;
        inputs.azimuthMotorVolts = azimuthAppliedVolts;
    }

    /////////// DRIVE MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setDriveVolts(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveMotor.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setDriveVelocity(double velocityMPS, double feedforward) {
        setDriveVolts(
            drivePID.calculate(
                driveMotor.getAngularVelocityRPM() * kCircumferenceMeters / 60, 
                velocityMPS) 
            + feedforward);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        drivePID.setPID(kP, kI, kD);
    }

    /////////// AZIMUTH MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setAzimuthVolts(double volts) {
        azimuthAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        azimuthMotor.setInputVoltage(azimuthAppliedVolts);
    }

    @Override
    public void setAzimuthPosition(Rotation2d position, double feedforward) {
        setAzimuthVolts(azimuthPID.calculate(azimuthMotor.getAngularPositionRad(), position.getRadians()) + feedforward);
    }

    @Override
    public void setAzimuthPID(double kP, double kI, double kD) {
        azimuthPID.setPID(kP, kI, kD);
    }
}
