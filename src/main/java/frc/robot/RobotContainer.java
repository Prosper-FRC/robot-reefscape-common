// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOKraken;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.Drive.DriveState;

import static frc.robot.subsystems.drive.DriveConstants.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Define subsystems
    private Drive drive;

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
                drive = new Drive( new Module[] {
                        new Module("FL", new ModuleIOKraken(kFrontLeft)),
                        new Module("FR", new ModuleIOKraken(kFrontRight)),
                        new Module("BL", new ModuleIOKraken(kBackLeft)),
                        new Module("BR", new ModuleIOKraken(kBackRight))
                    }, new GyroIOPigeon2());
                break;
            case SIM:
                // Instantiate subsystems that simulate actual hardware (IOSim modules)
                drive = new Drive( new Module[] {
                    new Module("FL", new ModuleIOSim()),
                    new Module("FR", new ModuleIOSim()),
                    new Module("BL", new ModuleIOSim()),
                    new Module("BR", new ModuleIOSim())
                }, new GyroIO(){});
                break;
            default:
                // Instantiate subsystems that are driven by playback of recorded sessions. (IO modules)
                drive = new Drive( new Module[] {
                    new Module("FL", new ModuleIO(){}),
                    new Module("FR", new ModuleIO(){}),
                    new Module("BL", new ModuleIO(){}),
                    new Module("BR", new ModuleIO(){})
                }, new GyroIO(){});
                break;
        }

        // Instantiate subsystems that don't care about mode, or are non-AdvantageKit enabled.
        // ex: LEDs = new LEDSubsystem();

        // Instantiate your TeleopCommands and AutonCommands classes
        telopCommands = new TeleopCommands(/* pass subsystems here */);
        autonCommands = new AutonCommands(drive);
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
        
        drive.acceptJoystickInputs(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX());

        drive.setDefaultCommand(Commands.run(
            () -> drive.setDriveEnum(DriveState.TELEOP), drive));
        
        configureStateTriggers();
        configureButtonBindings();

    }

    public Command getTeleopCommand() {
        return new SequentialCommandGroup(
            // Commands to run on teleop go here.
        );
    }

    public Command getAutonomousCommand() {
        Commands.runOnce(() -> {drive.setDriveEnum(DriveState.AUTON);}, drive).schedule();

        return autoChooser.get();
    }

    private void configureStateTriggers() {
        new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> drive.resetAllEncoders()));
    }

    private void configureButtonBindings() {
        driverController.povUp().onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_UP)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));
            
        driverController.povRight().onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_RIGHT)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

        driverController.povDown().onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_DOWN)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

        driverController.povLeft().onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_LEFT)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

        driverController.b().onTrue(drive.setDriveStateCommandContinued(DriveState.DRIFT_TEST)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

        driverController.y().onTrue(drive.setDriveStateCommandContinued(DriveState.RIGHT_DEG)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

        driverController.a().onTrue(drive.characterizeDriveMotors()).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));
        
        driverController.x().onTrue(Commands.runOnce(() -> {drive.resetGyro();}));

        // FOR DEBUGGING PURPOSES AND SHOULD BE REMOVED DURING COMP
        driverController.rightBumper().onTrue(Commands.runOnce(() -> {drive.resetPose();}));
    }

}
