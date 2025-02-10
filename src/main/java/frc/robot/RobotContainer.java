// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.SensorIO;
import frc.robot.subsystems.intake.SensorIOLaserCAN;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
    private final Elevator elevator;
    private final Intake intake;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Define subsystems
    // ex: private final LEDSubsystem LEDs;

    // Define other utility classes
    private final AutonCommands autonCommands;
    private final TeleopCommands telopCommands;

    private LoggedDashboardChooser<Command> autoChooser;

    private final boolean useCompetitionBindings = true;

    public RobotContainer() {

        // If using AdvantageKit, perform mode-specific instantiation of subsystems.
        switch (Constants.kCurrentMode) {
            case REAL:
                // elevator = new Elevator(
                //     new ElevatorIOTalonFX(
                //         Constants.kCanbusName, 
                //         ElevatorConstants.kRoboElevatorHardware, 
                //         ElevatorConstants.kMotorConfiguration, 
                //         ElevatorConstants.kElevatorGains));
                elevator = new Elevator(new ElevatorIO() {});
                // intake = new Intake(
                //     new IntakeIOTalonFX(
                //         IntakeConstants.kRoboIntakeHardware, 
                //         IntakeConstants.kMotorConfiguration), 
                //     new SensorIORange());
                intake = new Intake(new IntakeIO(){}, new SensorIOLaserCAN(IntakeConstants.kSensorConfiguration));
                break;
            case SIM:
                elevator = new Elevator(
                    new ElevatorIOSim(ElevatorConstants.kRoboElevatorHardware,
                        ElevatorConstants.kSimulationConfiguration,
                        ElevatorConstants.kElevatorGains,
                        ElevatorConstants.kMinPositionMeters,
                        ElevatorConstants.kMaxPositionMeters,
                        0.02));
                intake = new Intake(
                    new IntakeIOSim(
                        IntakeConstants.kRoboIntakeHardware, 
                        IntakeConstants.kSimulationConfiguration, 
                        0.02), 
                    new SensorIO(){});
                break;
            default:
                elevator = new Elevator(new ElevatorIO() {});
                intake = new Intake(new IntakeIO(){}, new SensorIO(){});
                break;
        }

        // Instantiate subsystems that don't care about mode, or are non-AdvantageKit enabled.
        // ex: LEDs = new LEDSubsystem();

        // Instantiate your TeleopCommands and AutonCommands classes
        telopCommands = new TeleopCommands(/* pass subsystems here */);
        autonCommands = new AutonCommands(/* pass subsystems here */);
        try {
            autoChooser = new LoggedDashboardChooser<>("Auton Program", autonCommands.getAutoChooser());
            // Fill instant command with whatever your initial action is
            autoChooser.addDefaultOption("initActionZeroPath", new InstantCommand());
        } catch (Exception e) {
            autoChooser = new LoggedDashboardChooser<Command>("Auton Program");
            // Fill instant command with whatever your initial action is, to prepare for the case of failure
            autoChooser.addDefaultOption("initActionZeroPath", new InstantCommand());
        }


        // Pass subsystems to classes that need them for configuration


        // Create any Dashboard choosers (LoggedDashboardChooser, etc)

        // Configure controls (drivebase suppliers, DriverStation triggers, Button and other Controller bindings)

        configureStateTriggers();
        configureButtonBindings();
    }

    public Command getTeleopCommand() {
        return new SequentialCommandGroup(
            // Commands to run on teleop go here.
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private void configureStateTriggers() {


    }

    private void configureButtonBindings() {

    }
}