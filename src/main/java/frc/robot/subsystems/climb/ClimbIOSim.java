package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.climb.ClimbConstants.ClimbHardware;
import frc.robot.subsystems.climb.ClimbConstants.SimulationConfiguration;

public class ClimbIOSim implements ClimbIO{

  private final double kLoopPeriodSec;

  private final DCMotorSim kMotor;

  private double appliedVoltage = 0.0;

  public ClimbIOSim(ClimbHardware hardware, SimulationConfiguration configuration, double loopPeriodSec) {
    kMotor = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        configuration.motorType(), 
        configuration.measurementStdDevs(), 
        hardware.gearing()), 
      configuration.motorType());

    kLoopPeriodSec = loopPeriodSec;
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs){
    kMotor.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.absolutePosistion = Rotation2d.fromRotations(kMotor.getAngularPositionRotations());
    inputs.relativePosistion = Rotation2d.fromRotations(kMotor.getAngularPositionRotations());
    inputs.appliedVoltage = appliedVoltage;
    inputs.temperatureCelsius = 0.0;
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
  }

  @Override
  public void setVoltage(double volts){
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    kMotor.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPosistion(Rotation2d posisiton){
    kMotor.setAngle(posisiton.getRadians());
  }

  @Override
  public void resetPosistion(){
    kMotor.setAngle(0);
  }

  @Override
  public void stop(){
    kMotor.setInputVoltage(0.0);
  }
}
