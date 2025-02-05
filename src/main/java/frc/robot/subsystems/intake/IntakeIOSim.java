
package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final double kLoopPeriodSec = 0.02;

  private final DCMotorSim kIntake;

  private double appliedVoltage = 0.0;

  public IntakeIOSim() {
    kIntake = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        IntakeConstants.kSimulationConfiguration.kMotorType(), 
        IntakeConstants.kSimulationConfiguration.kMeasurementStdDevs(), 
        IntakeConstants.kGearing), 
      IntakeConstants.kSimulationConfiguration.kMotorType());
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    kIntake.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.velocityRotationsPerSec = kIntake.getAngularVelocityRPM() / 60.0;
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    kIntake.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
