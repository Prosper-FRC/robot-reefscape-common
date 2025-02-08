
package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeConstants.IntakeHardware;
import frc.robot.subsystems.intake.IntakeConstants.SimulationConfiguration;

public class IntakeIOSim implements IntakeIO {
  private final double kLoopPeriodSec;

  private final DCMotorSim kIntake;

  private double appliedVoltage = 0.0;

  public IntakeIOSim(IntakeHardware hardware, SimulationConfiguration configuration, double loopPeriodSec) {
    kIntake = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        configuration.motorType(), 
        configuration.measurementStdDevs(), 
        hardware.gearing()), 
      configuration.motorType());

    kLoopPeriodSec = loopPeriodSec;
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
