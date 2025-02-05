// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorGains;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHardware;
import frc.robot.subsystems.elevator.ElevatorConstants.SimulationConfiguration;

public class ElevatorIOSim implements ElevatorIO {
  private final double kLoopPeriodSec;

  private final ElevatorSim kElevator;

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

  public ElevatorIOSim(ElevatorHardware hardware,
    SimulationConfiguration configuration,
    ElevatorGains gains,
    double minPositionMeters,
    double maxPositionMeters,
    double loopPeriodSec) {
    kElevator = new ElevatorSim(
      configuration.kMotorType(),
      hardware.kGearing(),
      configuration.kCarriageMassKg(),
      configuration.kDrumRadiusMeters(),
      minPositionMeters,
      maxPositionMeters,
      configuration.kSimulateGravity(),
      configuration.kStartingHeightMeters(),
      configuration.kMeasurementStdDevs(),
      configuration.kMeasurementStdDevs());

    kLoopPeriodSec = loopPeriodSec;

    kProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      gains.kMaxVelocityMetersPerSecond(), 
      gains.kMaxAccelerationMetersPerSecondSquared()));

    kFeedback = new PIDController(gains.kP(), gains.kI(), gains.kD());

    kFeedforward = 
      new ElevatorFeedforward(gains.kS(), gains.kG(), gains.kV(), gains.kA());
    
    // Reset elevator model to initial configuration in case it wasn't already
    resetPosition();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    kElevator.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.positionMeters = kElevator.getPositionMeters();
    inputs.velocityMetersPerSec = kElevator.getVelocityMetersPerSecond();
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

    kElevator.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPosition(double goalPositionMeters) {
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
      setpoint = new TrapezoidProfile.State(kElevator.getPositionMeters(), 0.0);
      feedbackNeedsReset = false;
    }
    goal = new TrapezoidProfile.State(goalPositionMeters, 0.0);

    setpoint = kProfile.calculate(kLoopPeriodSec, setpoint, goal);

    double feedforwardEffort = kFeedforward.calculate(setpoint.velocity);
    double feedbackEffort = kFeedback.calculate(kElevator.getPositionMeters(), setpoint.position);

    Logger.recordOutput("Elevator/Feedback/FBEffort", feedbackEffort);
    Logger.recordOutput("Elevator/Feedback/FFEffort", feedforwardEffort);
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
    kElevator.setState(0.0, 0.0);
  }
}
