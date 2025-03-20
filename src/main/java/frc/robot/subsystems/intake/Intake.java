
package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;
import frc.robot.utils.visualizers.PivotVisualizer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Intake extends SubsystemBase {
  /** List of voltage setpoints for the intake in voltage */
  public enum RollerGoal {
    kIntakeCoral(() -> 2.0),
    kIntakeAlgae(() -> -5.0),
    kScoreCoral(() -> -4.0),
    kAutonScoreCoral(() -> 4.0),
    kScoreAlgae(() -> -5.0),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    custom(new LoggedTunableNumber("Intake/Feedback/RollerSetpointVolts", 0.0));

    private DoubleSupplier goalVoltage;

    RollerGoal(DoubleSupplier goalVoltage) {
      this.goalVoltage = goalVoltage;
    }

    public double getGoalVolts() {
      return this.goalVoltage.getAsDouble();
    }
  }

    // TODO Remove this after full extrapolation
    /** List of position setpoints for the pivot */
    public enum PivotGoal {
      kStowScore(() -> Rotation2d.fromDegrees(64.0)),
      kStowPickup(() -> Rotation2d.fromDegrees(54.0)),
      kIntakeReef(() -> Rotation2d.fromDegrees(5.0)),
      kIntakeGround(() -> Rotation2d.fromDegrees(-55.5)),
      kProcessorScore(() -> Rotation2d.fromDegrees(-30.0)),
      kScore(() -> Rotation2d.fromDegrees(40.0)),
      kBargeScore(() -> Rotation2d.fromDegrees(46.0)),
      /** Custom setpoint that can be modified over network tables; Useful for debugging */
      custom(() -> Rotation2d.fromDegrees(
        new LoggedTunableNumber("Intake/Feedback/PivotSetpointDegrees", 0.0).get()));
  
      private Supplier<Rotation2d> goalPosition;
  
      PivotGoal(Supplier<Rotation2d> goalPosition) {
        this.goalPosition = goalPosition;
      }
  
      public Rotation2d getGoalPosition() {
        return this.goalPosition.get();
      }
    }

  /*
   * TODO At some point this can be moved out in favor of using some form of higher level 
   * robot state control
   */
  /** Specify which gamepiece the mechanism is attempting to manipulate */
  public enum Gamepiece {
    kCoral,
    kAlgae
  }

  private final IntakeIO kRollerHardware;
  private final IntakeIOInputsAutoLogged kRollerInputs = new IntakeIOInputsAutoLogged();

  private final SensorIO kSensor;
  private final SensorIOInputsAutoLogged kSensorInputs = new SensorIOInputsAutoLogged();

  private boolean detectedGamepiece = false;
  private LinearFilter ampFilter = LinearFilter.movingAverage(
    IntakeConstants.kLinearFilterSampleCount);

  private RollerGoal rollerGoal = null;
  private PivotGoal pivotGoal = null; // TODO Remove this after full extrapolation

  // The default gamepiece is coral, this is because we will start preloaded with coral and will
  // assume throughout the rest of the code the robot will always default to scoring coral
  @AutoLogOutput(key ="Intake/Gamepiece")
  private Gamepiece selectedGamepiece = Gamepiece.kCoral;

  // Object used to visualize the mechanism over network tables, useful in simulation
  private final PivotVisualizer kPivotVisualizer;

  private final LoggedNetworkBoolean kOverrideDetectGamepiece = 
    new LoggedNetworkBoolean("Intake/OverrideDetectGamepiece", false);

  public Intake(IntakeIO hardwareIO, SensorIO sensorIO) {
    kRollerHardware = hardwareIO;
    kSensor = sensorIO;

    kPivotVisualizer = new PivotVisualizer(
      "Intake/PivotVisualizer", 
      IntakeConstants.kPivotVisualizerConfiguration, 
      4.0, 
      new Color8Bit(Color.kBlue));
  }

  @Override
  public void periodic() {
    kRollerHardware.updateInputs(kRollerInputs);
    Logger.processInputs("Intake/Inputs/Rollers", kRollerInputs);
    kSensor.updateInputs(kSensorInputs);
    Logger.processInputs("Intake/Inputs/Sensor", kSensorInputs);

    // Stop and clear goal if disabled. Used if copilot is still pressing button to command
    // intake when the disabled key is pressed
    if (DriverStation.isDisabled()) {
      stop(true, true);
    }

    double ampFilterCalculation = ampFilter.calculate(kRollerInputs.statorCurrentAmps);

    if (selectedGamepiece == Gamepiece.kCoral) {
      // If the CANrange disconnects we can use motor current to detect when we have a coral
      // TODO Test this fallback to see if it actually works
      if (!kOverrideDetectGamepiece.get()) {
        if (kSensorInputs.isConnected) {
          detectedGamepiece = kSensorInputs.detectsObject;
        } else {
          // Checks for spike in amperage, and if greater than the value then
          // the intake motor probbaly has the coral
          detectedGamepiece = ampFilterCalculation > IntakeConstants.kCoralAmpFilterThreshold;
        }
      } else {
        detectedGamepiece = true;
      }
    } else if (selectedGamepiece == Gamepiece.kAlgae) {
      if (!kOverrideDetectGamepiece.get()) {
        // Checks for spike in amperage, and if greater than the value then
        // the intake motor probbaly has the algae
        detectedGamepiece = ampFilterCalculation > IntakeConstants.kAlgaeAmpFilterThreshold;
      } else {
        detectedGamepiece = true;
      }
    } else {
      System.out.println("INTAKE: selectedGamepiece is invalid");
    }

    if (rollerGoal != null) {
      setRollerVoltage(rollerGoal.getGoalVolts());
      Logger.recordOutput("Intake/RollerGoal", rollerGoal);
    } else {
      Logger.recordOutput("Intake/RollerGoal", "NONE");
    }

    if (rollerGoal != null) {
      if (detectedGamepiece() && (rollerGoal.equals(RollerGoal.kIntakeAlgae) || rollerGoal.equals(RollerGoal.kIntakeCoral))) {
        stop(true, false);
      }
    }

    // The visualizer needs to be periodically fed the current position of the mechanism
    kPivotVisualizer.updatePosition(getPivotPosition().times(-1.0));
  }

  /**
   * Sets the voltage goal of the roller mechanism, logic runs in subsystem periodic method
   * 
   * @param desiredGoal The desired voltage goal
   */
  public void setRollerGoal(RollerGoal desiredGoal) {
    rollerGoal = desiredGoal;
  }

  /**
   * Sets the voltage goal of the pivo mechanism, logic runs in subsystem periodic method
   * 
   * @param desiredGoal The desired voltage goal
   */
  public void setPivotGoal(PivotGoal desiredGoal) {
    pivotGoal = desiredGoal;
  }

  /**
   * Sets the desired gamepiece of the subsystem, used when checking for if the robot has a
   * gamepiece or not
   * 
   * @param desiredGamepiece The gamepiece type
   */
  public void selectGamepiece(Gamepiece desiredGamepiece) {
    if (desiredGamepiece != null) {
      selectedGamepiece = desiredGamepiece;
    } else {
      System.out.println("INTAKE: Invalid gamepiece selected");
      selectedGamepiece= Gamepiece.kCoral;
    }
  }

  /** Stops the motor */
  public void stop(boolean stopRollers, boolean stopPivot) {
    if (stopRollers) {
      rollerGoal = null;
      kRollerHardware.stop();
    }
  }

  /**
   * Sets the voltage of the roller motor
   * 
   * @param voltage
   */
  public void setRollerVoltage(double voltage) {
    kRollerHardware.setVoltage(voltage);
  }

  /**
   * Sets the voltage of the pivot motor
   * 
   * @param voltage
   */
  public void setPivotVoltage(double voltage) {
    // TODO Remove this method
  }

  public void setPivotPosition(Rotation2d position) {
    // TODO Remove this method
  }

  /**
   * Sets the vertical position of the mechanism on the visuzlier, useful as the
   * pivot moves with the elevator
   * 
   * @param positionMeters
   */
  public void setVisualizerVerticalPosition(double positionMeters) {
    kPivotVisualizer.setRootVerticalPositionMeters(positionMeters);
  }

  /**
   * The subsystem runs a linear filter that accepts the motor's current and 
   * compares the moving average against a threshold. If that threshold is exceeded, 
   * it is very likely that the motor has a gamepiece (or is jammed)
   * 
   * @return If the motor thinks it has a gamepiece
   */
  @AutoLogOutput(key = "Intake/detectedGamepiece")
  public boolean detectedGamepiece() {
    return detectedGamepiece;
  }

  /**
   * Compute the error based off of our current position and current goal
   * 
   * @return The computed error in degrees
   */
  @AutoLogOutput(key = "Pivot/Feedback/ErrorDegrees")
  public double getPivotErrorDegrees() {
    // TODO Remove this method
    return 0.0;
  }

  /**
   * @return If the pivot is at its desired goal yet
   */
  @AutoLogOutput(key = "Pivot/Feedback/AtGoal")
  public boolean pivotAtGoal() {
    return Math.abs(getPivotErrorDegrees()) < IntakeConstants.kPivotPositionTolerance.getDegrees();
  }

  /**
   * @return The position of the pivot
   */
  public Rotation2d getPivotPosition() {
    // TODO Remove this method
    return new Rotation2d();
  }

  public Double getStatorCurrent() {
    return kRollerInputs.statorCurrentAmps;
  }
}