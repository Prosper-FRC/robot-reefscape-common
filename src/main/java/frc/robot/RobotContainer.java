// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.climb.Climb.ClimbVoltageGoal;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
    private final Climb climb;

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
                // Instantiate subsystems that operate actual hardware (Hardware controller based modules)
                climb = new Climb(
                    new ClimbIOTalonFX(
                        ClimbConstants.kLeadMotorHardware, 
                        ClimbConstants.kLeadMotorConfiguration, 
                        ClimbConstants.kMotorGains, 
                        ClimbConstants.kStatusSignalUpdateFrequency),
                    new ClimbIOTalonFX(
                        ClimbConstants.kFollowerMotorHardware, 
                        ClimbConstants.kFollowMotorConfiguration, 
                        ClimbConstants.kMotorGains, 
                        ClimbConstants.kStatusSignalUpdateFrequency));
                break;
            case SIM:
                // Instantiate subsystems that simulate actual hardware (IOSim modules)
                climb = new Climb(new ClimbIOSim(
                    0.02,
                    ClimbConstants.kLeadMotorHardware,
                    ClimbConstants.kSimulationConfiguration,
                    ClimbConstants.kMotorGains));
                break;
            default:
                // Instantiate subsystems that are driven by playback of recorded sessions. (IO modules)
                climb = new Climb(new ClimbIO[]{new ClimbIO(){}});
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
        operatorController.a()
            .whileTrue(
                Commands.run(() -> climb.setGoalVoltage(ClimbVoltageGoal.kGrab), climb))
            .whileFalse(
                Commands.runOnce(climb::stop, climb));

        operatorController.b()
            .whileTrue(
                Commands.run(() -> climb.setGoalVoltage(ClimbVoltageGoal.kRelease), climb))
            .whileFalse(
                Commands.runOnce(climb::stop, climb));
    }
}