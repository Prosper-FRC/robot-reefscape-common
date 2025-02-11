// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;

public class Elevator extends SubsystemBase {
  /** List of position setpoints for the elevator in meters */
  public enum ElevatorGoal {
    kL4Coral(() -> Units.inchesToMeters(60.0)),
    kL3Coral(() -> 0.84),
    kL2Coral(() -> 0.41),
    kL1Coral(() -> Units.inchesToMeters(8.0)),
    /** Stow the elevator during transit */
    kStow(() -> Units.inchesToMeters(4.0)),
    /** Position for intaking from the coral station */
    kIntake(() -> Units.inchesToMeters(8.0)),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    custom(new LoggedTunableNumber("Elevator/Custom", 0.0));

    private DoubleSupplier goalMeters;

    ElevatorGoal(DoubleSupplier goalMeters) {
      this.goalMeters = goalMeters;
    }

    public double getGoalMeters() {
      return this.goalMeters.getAsDouble();
    }
  }

  private final ElevatorIO kHardware;
  private final ElevatorIOInputsAutoLogged kInputs = new ElevatorIOInputsAutoLogged();
  private final MagneticSensorIO kSensor;
  private final MagneticSensorIOInputsAutoLogged kSensorInputs = new MagneticSensorIOInputsAutoLogged();

  private ElevatorGoal currentElevaotrGoal = null;

  private double currentElevatorGoalPositionMeters = 0.0;

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Elevator/Gains/kP", ElevatorConstants.kElevatorGains.p());
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Elevator/Gains/kI", ElevatorConstants.kElevatorGains.i());
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Elevator/Gains/kD", ElevatorConstants.kElevatorGains.d());
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Elevator/Gains/kS", ElevatorConstants.kElevatorGains.s());
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Elevator/Gains/kV", ElevatorConstants.kElevatorGains.v());
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Elevator/Gains/kA", ElevatorConstants.kElevatorGains.a());
  private final LoggedTunableNumber kG =
      new LoggedTunableNumber("Elevator/Gains/kG", ElevatorConstants.kElevatorGains.g());
  private final LoggedTunableNumber kMaxVelocity =
      new LoggedTunableNumber(
          "Elevator/MotionMagic/kMaxVelocity", 
          ElevatorConstants.kElevatorGains.maxVelocityMetersPerSecond());
  private final LoggedTunableNumber kMaxAcceleration =
      new LoggedTunableNumber(
          "Elevator/MotionMagic/kMaxAcceleration", 
          ElevatorConstants.kElevatorGains.maxAccelerationMetersPerSecondSquared());

  // Object used to visualize the mechanism over network tables, useful in simulation
  private final ElevatorVisualizer kVisualizer;

  private final LinearFilter kHomingFilterAmps = LinearFilter.movingAverage(5);

  private LoggedNetworkBoolean simulatedHomingSensorActivator = 
    new LoggedNetworkBoolean("Elevator/hasHomed", false);

  private boolean isHoming = false;
  private boolean hasHomed = false;

  public Elevator(ElevatorIO io, MagneticSensorIO ioSensor) {
    kHardware = io;
    kSensor = ioSensor;
    kVisualizer = new ElevatorVisualizer(getPositionMeters());
  }

  @Override
  public void periodic() {
    kHardware.updateInputs(kInputs);
    Logger.processInputs("Elevator/Inputs", kInputs);
    kSensor.updateInputs(kSensorInputs);
    Logger.processInputs("Elevator/Inputs/Sensor", kSensorInputs);

    // Stop and clear goal if disabled. Used if copilot is still pressing button to command
    // elevator to go to a setpoint when the disabled key is pressed
    if (DriverStation.isDisabled()) {
      stop();
    }

    double elevatorAverageCurrentAmps = kHomingFilterAmps.calculate(kInputs.statorCurrentAmps);

    // Check if we are homing 
    if (!isHoming) {
      // Run the elevator goal
      if (currentElevaotrGoal != null) {
        currentElevatorGoalPositionMeters = currentElevaotrGoal.getGoalMeters();
        setPosition(currentElevatorGoalPositionMeters);
        kVisualizer.setGoalLine(currentElevatorGoalPositionMeters, atGoal());
  
        Logger.recordOutput("Elevator/Goal", currentElevaotrGoal);
      } else {
        Logger.recordOutput("Elevator/Goal", "NONE");
        kVisualizer.setGoalLine(0.0, false);
      }

      // Check if elevator is attempting to move beyond its limitations
      if (getPositionMeters() > ElevatorConstants.kMaxPositionMeters
          && kInputs.appliedVoltage > 0.0) {
        kHardware.stop();
      } else if (getPositionMeters() < ElevatorConstants.kMinPositionMeters
          && kInputs.appliedVoltage < 0.0) {
        kHardware.stop();
      } else {
        // Do nothing if limits are not reached
      }
    } else {
      // If we are homing...
      if (RobotBase.isReal()) {
        // If the magnetic sensor is activated, we've completed homing
        if (kSensorInputs.isActivated) {
          resetPosition();
          stop();
          hasHomed = true;
          isHoming = false;
        } else {
        // Otherwise, continue to run the elevator downwards
          kHardware.setVoltage(-2.0);
          hasHomed = false;
          isHoming = true;
        }
      } else {
        // If we're in sim, simulate via button that can be toggled on dashboard
        if (simulatedHomingSensorActivator.get()) {
          resetPosition();
          stop();
          hasHomed = true;
          isHoming = false;
        } else {
          kHardware.setVoltage(-2.0);
          hasHomed = false;
          isHoming = true;
        }
      }
    }

    Logger.recordOutput("Elevator/isHoming", isHoming);
    Logger.recordOutput("Elevator/hasHomed", hasHomed);

    // This says that if the value is changed in the advantageScope tool,
    // Then we change the values in the code. Saves deploy time.
    // More found in prerequisites slide
    LoggedTunableNumber.ifChanged(
      hashCode(),
      () -> {
        kHardware.setGains(
            kP.get(), kI.get(), kD.get(), kS.get(), kG.get(), kV.get(), kA.get());
      },
      kP,
      kI,
      kD,
      kS,
      kV,
      kA,
      kG);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          kHardware.setMotionMagicConstraints(kMaxVelocity.get(), kMaxAcceleration.get());
        },
        kMaxVelocity,
        kMaxAcceleration);

    // The visualizer needs to be periodically fed the current position of the mechanism
    kVisualizer.updateElevatorPosition(getPositionMeters());
  }

  /**
   * Sets the position goal of the mechanism, logic runs in subsystem periodic method
   * 
   * @param desiredGoal The desired position goal
   */
  public void setGoal(ElevatorGoal desiredGoal) {
    currentElevaotrGoal = desiredGoal;
  }

  /**
   * Sets the voltage of the motor,
   * 
   * @param voltage
   */
  public void setVoltage(double voltage) {
    // Notice how we are not checking if position control is running, it is up to the caller
    // to check for this before calling this method (I recommend calling this subsystem's stop() 
    // method after completing any action)
    if (getPositionMeters() > ElevatorConstants.kMaxPositionMeters
        && voltage > 0.0) {
      return;
    } else if (getPositionMeters() < ElevatorConstants.kMinPositionMeters
        && voltage < 0.0) {
      return;
    } else {
      kHardware.setVoltage(voltage);
    }
  }

  /** Stops the mechanism */
  public void stop() {
    currentElevaotrGoal = null;
    kHardware.stop();
  }

  /** Reset the mechanism's encoder to 0 meters */
  public void resetPosition() {
    kHardware.resetPosition();
  }

  /**
   * Set the position goal of the hardware layer
   * 
   * @param positionGoalMeters The position goal in meters
   */
  public void setPosition(double positionGoalMeters) {
    positionGoalMeters = MathUtil.clamp(
      positionGoalMeters, ElevatorConstants.kMinPositionMeters, ElevatorConstants.kMaxPositionMeters);
    kHardware.setPosition(positionGoalMeters);
  }

  /** 
   * Initiates the homing sequence, note that this will override all other actions until completed or
   * cancelled
   */
  public void homeElevator() {
    isHoming = true;
    hasHomed = false;
    // Pass in null goal so the elevator doesn't go anywhere if homing is cancelled
    setGoal(null);
  }

  /** Cancels the homing sequence */
  public void cancelHoming() {
    isHoming = false;
  }

  /**
   * Check if the elevator has completed its homing routine, note that this will reset 
   * when the homing sequence is recalled
   * 
   * @return If the elevator has completed homing
   */
  public boolean hasHomed() {
    return hasHomed;
  }

  /**
   * Set the elevator motor to brake or coast mode
   * 
   * @param enable If the brake should be enabled (True = brake | False = coast)
   */
  public void enableElevatorBrake(boolean enable) {
    kHardware.setBrakeMode(enable);
  }

  /**
   * Compute the error based off of our current position and current goal
   * 
   * @return The computed error in meters
   */
  @AutoLogOutput(key = "Elevator/Feedback/ErrorMeters")
  public double getErrorMeters() {
    return currentElevatorGoalPositionMeters - getPositionMeters();
  }

  /**
   * @return If the elevator is at its desired goal yet
   */
  @AutoLogOutput(key = "Elevator/Feedback/AtGoal")
  public boolean atGoal() {
    return Math.abs(getErrorMeters()) < ElevatorConstants.kPositionToleranceMeters;
  }

  /**
   * @return The position of the linear mechanism in meters
   */
  public double getPositionMeters() {
    return kInputs.positionMeters;
  }

  /**
   * It may be useful to know what the goal of the feedback controller is
   * 
   * @return The goal position of the linear mechanism in meters (typically set in periodic)
   */
  @AutoLogOutput(key = "Elevator/Feedback/GoalMeters")
  public double getPositionGoalMeters() {
    return currentElevatorGoalPositionMeters;
  }
}