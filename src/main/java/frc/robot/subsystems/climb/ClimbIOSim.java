// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.climb.ClimbConstants.ClimbGains;
import frc.robot.subsystems.climb.ClimbConstants.ClimbHardware;
import frc.robot.subsystems.climb.ClimbConstants.ClimbSimulationConfiguration;

public class ClimbIOSim implements ClimbIO {
  private final double kLoopPeriodSec;

  private final SingleJointedArmSim kPivot;

  // Create and use the feedback and feedforware controllers in here since we 
  // are using the internal motor controllers
  private TrapezoidProfile kProfile;

  private final PIDController kFeedback;

  // Feedforward still a constant, but we have to reinstantiate the model in 
  // order to change the gains
  private ElevatorFeedforward kFeedforward;

  // Final goal of the mechanism
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  // Setpoint to go to following the profile
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private double appliedVoltage = 0.0;

  // Used to determine the current control mode of the simulation hardware
  private boolean feedbackNeedsReset = false;
  private boolean closedLoopControl = false;

  public ClimbIOSim(
    double loopPeriodSec,
    ClimbHardware hardware,
    ClimbSimulationConfiguration configuration, 
    ClimbGains gains) {

    kLoopPeriodSec = loopPeriodSec;

    kPivot = new SingleJointedArmSim(
      configuration.motorType(),
      hardware.gearing(),
      configuration.momentOfInertiaJKgMetersSquared(),
      configuration.ligamentLengthMeters(),
      configuration.minPosition().getRadians(),
      configuration.maxPosition().getRadians(), 
      configuration.simulateGravity(), 
      configuration.initialPosition().getRadians(), 
      configuration.meadurementStdDevs(),
      configuration.meadurementStdDevs());

    kProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      gains.maxVelocityMetersPerSecond(), 
      gains.maxAccelerationMetersPerSecondSquared()));

    kFeedback = new PIDController(gains.p(), gains.i(), gains.d());

    kFeedforward = 
      new ElevatorFeedforward(gains.s(), gains.g(), gains.v(), gains.a());
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    kPivot.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.position = Rotation2d.fromRadians(kPivot.getAngleRads());
    inputs.velocityUnitsPerSec = Rotation2d.fromRadians(kPivot.getVelocityRadPerSec());
    inputs.appliedVoltage = appliedVoltage;
    // These are 0 cause we don't care able these in simulation
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoopControl = false;
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);

    kPivot.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPosition(Rotation2d goalPosition) {
    // Check if we weren't in closed loop, if we weren't reset the motion 
    // profile. Recall that the profile should be reset before each major 
    // movement of the mechanism. We don't need this logic in the real IO 
    // since it's handled by the controller
    if (!closedLoopControl) {
      feedbackNeedsReset = true;
      closedLoopControl = true;
    }
    if (feedbackNeedsReset) {
      kFeedback.reset();
      setpoint = new TrapezoidProfile.State(kPivot.getAngleRads(), 0.0);
      feedbackNeedsReset = false;
    }
    goal = new TrapezoidProfile.State(goalPosition.getRadians(), 0.0);

    setpoint = kProfile.calculate(kLoopPeriodSec, setpoint, goal);

    double feedforwardEffort = kFeedforward.calculate(setpoint.velocity);
    double feedbackEffort = kFeedback.calculate(kPivot.getAngleRads(), setpoint.position);

    // In the unlikely event that there are multiple classes created, we need a unique
    // name to distinguish between them on network tables
    String rootLogKey = getClass().getName() + "@" + Integer.toHexString(hashCode());

    Logger.recordOutput(rootLogKey + "/Feedback/FBEffort", feedbackEffort);
    Logger.recordOutput(rootLogKey + "/Feedback/FFEffort", feedforwardEffort);
    setVoltage(feedbackEffort + feedforwardEffort);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setGains(double p, double i, double d, double s, double g, double v, double a)  {
    kFeedback.setPID(p, i, d);
    kFeedforward = new ElevatorFeedforward(s, g, v, a);
  }

  @Override
  public void setMotionMagicConstraints(double maxVelocity, double maxAcceleration) {
    kProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  }

  @Override
  public void resetPosition() {
    kPivot.setState(0.0, 0.0);
  }
}
