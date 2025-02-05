// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

// How to use the visualizer with AdvantageScope:
// https://docs.advantagescope.org/tab-reference/mechanism

/** Class to handle graphical visualization of an elevator mechanism. */
public class ElevatorVisualizer {
  // Standard path that the visual data should be published to
  private final String LOG_KEY = "Elevator/Visualier";

  private final double kStageOneMinimumLengthMeters = 1.0470896;
  private final double kCarriageMinimumLengthMeters = 0.2469896;
  private final double kScoreMinimumLengthMeters = 0.3469896;

  // Offsets for the different stages of the elevator, used to graphically differentiate the 
  // stages of the elevator visualizer
  private final double kElevatorStageOffset = 0.5;
  private final double kStructureOffset = kElevatorStageOffset - 0.04;
  private final double kCarriageOffset = kElevatorStageOffset + 0.04;

  /** The field that all mechanism stages are appended to. Units are in meters */
  private LoggedMechanism2d elevatorVisualField = new LoggedMechanism2d(1.0, 3.0);
  // Structure root is the part of the elevator that doesn't move
  private LoggedMechanismRoot2d structureRoot = elevatorVisualField.getRoot("structureRoot", kStructureOffset, 0.0);
  // The elevator stage and carraige are going to move semi-independently of each other on the visualizer
  private LoggedMechanismRoot2d elevatorRoot = elevatorVisualField.getRoot("elevatorRoot", kElevatorStageOffset, 0.0);
  private LoggedMechanismRoot2d carriageRoot = elevatorVisualField.getRoot("carriageRoot", kCarriageOffset, 0.0);

  // The "structure" is the part that the first stage uses to move upwards
  private LoggedMechanismLigament2d structure = 
    structureRoot.append(
      new LoggedMechanismLigament2d("structure", kStageOneMinimumLengthMeters, 90.0));
  private LoggedMechanismLigament2d elevatorFirstStage = 
    elevatorRoot.append(
      new LoggedMechanismLigament2d("elevator", kStageOneMinimumLengthMeters, 90.0, 
      4.0, new Color8Bit(Color.kWhite)));
  private LoggedMechanismLigament2d elevatorCarriage = 
    carriageRoot.append(
      new LoggedMechanismLigament2d("carriage", kCarriageMinimumLengthMeters, 90.0, 
      4.0, new Color8Bit(Color.kBlue)));
  // End-affector is appended to the carriage
  private LoggedMechanismLigament2d scorer = 
    elevatorCarriage.append(
      new LoggedMechanismLigament2d("scorer", kScoreMinimumLengthMeters, 0.0, 
      4.0, new Color8Bit(Color.kOrange)));
  private LoggedMechanismLigament2d box = 
    scorer.append(
      new LoggedMechanismLigament2d("box", kScoreMinimumLengthMeters, -120.0));

  // Vertical positions for visualization
  // Standard path that the visual data should be published to
  private final String MEASURE_LOG_KEY = "Elevator/VisualierMeasure";

  /** The field that shows measurements for reference */
  private LoggedMechanism2d mesaurementVisualField = new LoggedMechanism2d(1.0, 3.0);

  private LoggedMechanismRoot2d halfMeterRoot = mesaurementVisualField.getRoot("halfMeterRoot", 0.0, 0.5);
  private LoggedMechanismRoot2d oneMeterRoot = mesaurementVisualField.getRoot("oneMeterRoot", 0.0, 1.0);
  private LoggedMechanismRoot2d oneHalfMeterRoot = mesaurementVisualField.getRoot("oneHalfMeterRoot", 0.0, 1.5);
  private LoggedMechanismRoot2d maxLengthRoot = mesaurementVisualField.getRoot(
    "maxLengthRoot", 0.0, ElevatorConstants.kMaxPositionMeters);
  LoggedMechanismRoot2d setpointRoot = mesaurementVisualField.getRoot("mainSetpointRoot", 0.0, 0.0);

  // Negative symbol cause that's just how the visualizer works
  private final double kLineLength = -0.4;

  // The angle is 180 because we want the lines to appear like horizontal tick marks
  private LoggedMechanismLigament2d halfMeterLine = 
    halfMeterRoot.append(
      new LoggedMechanismLigament2d("halfMeterLine", kLineLength, 180.0));
  private LoggedMechanismLigament2d oneMeterLine = 
    oneMeterRoot.append(
      new LoggedMechanismLigament2d("oneMeterLine", kLineLength, 180.0));
  private LoggedMechanismLigament2d oneHalfMeterLine = 
    oneHalfMeterRoot.append(
      new LoggedMechanismLigament2d("oneHalfMeterLine", kLineLength, 180.0));
  private LoggedMechanismLigament2d maxLengthLine = 
    maxLengthRoot.append(
      new LoggedMechanismLigament2d("maxLengthLine", kLineLength, 180.0));
  private LoggedMechanismLigament2d setpointLine = 
    setpointRoot.append(
      new LoggedMechanismLigament2d("mainSetpointLine", kLineLength, 180.0));

  /**
   * Creates a new visualizer
   *
   * @param initialPosition The starting position of the arm
   */
  public ElevatorVisualizer(double initialPositionMeters) {
    // Set the visual details of the endaffector here so intellisense doesn't mark this as an
    // unused variable (IE Don't worry about this)
    box.setLineWeight(4.0);
    box.setColor(new Color8Bit(Color.kOrange));

    structure.setLineWeight(4.0);
    structure.setColor(new Color8Bit(Color.kAqua));

    halfMeterLine.setLineWeight(2.0);
    halfMeterLine.setColor(new Color8Bit(Color.kLime));
    oneMeterLine.setLineWeight(2.0);
    oneMeterLine.setColor(new Color8Bit(Color.kLime));
    oneHalfMeterLine.setLineWeight(2.0);
    oneHalfMeterLine.setColor(new Color8Bit(Color.kLime));
    maxLengthLine.setLineWeight(2.0);
    maxLengthLine.setColor(new Color8Bit(Color.kFirstRed));

    Logger.recordOutput(LOG_KEY, elevatorVisualField);
    Logger.recordOutput(MEASURE_LOG_KEY, mesaurementVisualField);
  }

  /**
   * Updates the position of the elevator on the visualizer
   * 
   * @param positionMeters The current position of the elevator mechanism
   */
  public void updateElevatorPosition(double positionMeters) {
    // Cascading elevator has both stages move at the same time, meaning that we must divide
    // the new position of the elevator by the number of stages
    double elevatorFirstStageNewPosition = positionMeters / 2.0;
    // The carriage's position is still the same however, since it is the final stage
    double carriageNewPosition = positionMeters;

    elevatorRoot.setPosition(kElevatorStageOffset, elevatorFirstStageNewPosition);
    carriageRoot.setPosition(kCarriageOffset, carriageNewPosition);

    Logger.recordOutput(LOG_KEY, elevatorVisualField);
  }

  /**
   * Display a line in the 2D visualizer that shows the goal setpoint
   * 
   * @param setpointMeters The vertical position setpoint in meters that should be displayed
   * @param atGoal Whether the mechanism is currently at the goal or not
   */
  public void setGoalLine(double setpointMeters, boolean atGoal) {
    setpointRoot.setPosition(0.0, setpointMeters);
    setpointLine.setLineWeight(2.0);
    if (atGoal) {
      setpointLine.setColor(new Color8Bit(Color.kLimeGreen));
    } else {
      setpointLine.setColor(new Color8Bit(Color.kOrange));
    }

    Logger.recordOutput(MEASURE_LOG_KEY, mesaurementVisualField);
  }

  /**
   * @return The current length that the first stage mechanism visual is set to
   */
  @AutoLogOutput(key = LOG_KEY + "/FirstStageLength")
  public double getFirstStageLength() {
    return elevatorFirstStage.getLength();
  }

  /**
   * @return The current length that the carriage mechanism visual is set to
   */
  @AutoLogOutput(key = LOG_KEY + "/SecondStageLength")
  public double getCarriageLength() {
    return elevatorCarriage.getLength();
  }
}
