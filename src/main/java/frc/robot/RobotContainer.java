// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOKraken;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPV;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.Drive.DriveState;

import static frc.robot.subsystems.drive.DriveConstants.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Define subsystems
    private final Drive robotDrive;

    // Define other utility classes
    private final AutonCommands autonCommands;
    private final TeleopCommands telopCommands;

    private LoggedDashboardChooser<Command> autoChooser;

    private final boolean useCompetitionBindings = false;

    public RobotContainer() {

        // If using AdvantageKit, perform mode-specific instantiation of subsystems.
        switch (Constants.kCurrentMode) {
            case REAL:
                // Instantiate subsystems that operate actual hardware (Hardware controller based modules)
                robotDrive = new Drive( new Module[] {
                    new Module("FL", new ModuleIOKraken(kFrontLeftHardware )),
                    new Module("FR", new ModuleIOKraken(kFrontRightHardware)),
                    new Module("BL", new ModuleIOKraken(kBackLeftHardware  )),
                    new Module("BR", new ModuleIOKraken(kBackRightHardware ))
                }, new GyroIOPigeon2(), new Vision(new VisionIO[] {
                    new VisionIOPV(VisionConstants.kRightCamName, VisionConstants.kRightCamTransform), 
                    new VisionIOPV(VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform)
                }));
                break;
            case SIM:
                // Instantiate subsystems that simulate actual hardware (IOSim modules)
                robotDrive = new Drive( new Module[] {
                    new Module("FL", new ModuleIOSim()),
                    new Module("FR", new ModuleIOSim()),
                    new Module("BL", new ModuleIOSim()),
                    new Module("BR", new ModuleIOSim())
                }, new GyroIO() {}, new Vision(new VisionIO[] {
                    new VisionIOPV(VisionConstants.kRightCamName, VisionConstants.kRightCamTransform), 
                    new VisionIOPV(VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform)
                }));
                break;
            default:
                // Instantiate subsystems that are driven by playback of recorded sessions. (IO modules)
                robotDrive = new Drive( new Module[] {
                    new Module("FL", new ModuleIO() {}),
                    new Module("FR", new ModuleIO() {}),
                    new Module("BL", new ModuleIO() {}),
                    new Module("BR", new ModuleIO() {})
                }, new GyroIO() {}, new Vision(new VisionIO[] {
                    new VisionIO() {}, new VisionIO() {}
                }));
                break;
        }

        // Instantiate subsystems that don't care about mode, or are non-AdvantageKit enabled.
        // ex: LEDs = new LEDSubsystem();

        // Instantiate your TeleopCommands and AutonCommands classes
        telopCommands = new TeleopCommands(/* pass subsystems here */);
        autonCommands = new AutonCommands(robotDrive);
        try {
            autoChooser = new LoggedDashboardChooser<>("Auton Program", autonCommands.getAutoChooser());
            // Fill instant command with whatever your initial action is
            autoChooser.addDefaultOption("initActionZeroPath", new InstantCommand());
        } catch (Exception e) {
            autoChooser = new LoggedDashboardChooser<Command>("Auton Program");
            // Fill instant command with whatever your initial action is, to prepare for the case of failure
            autoChooser.addDefaultOption("PATHS FAILED: initActionZeroPath", new InstantCommand());
        }



        // Pass subsystems to classes that need them for configuration
        robotDrive.acceptJoystickInputs(
            () -> - driverController.getLeftY(),
            () -> - driverController.getLeftX(),
            () -> - driverController.getRightX(),
            () -> driverController.getHID().getPOV());

        // Create any Dashboard choosers (LoggedDashboardChooser, etc)

        // Configure controls (drivebase suppliers, DriverStation triggers, Button and other Controller bindings)

        configureStateTriggers();
        configureButtonBindings();
    }

    public Command getTeleopCommand() {
        return new SequentialCommandGroup(
            robotDrive.setDriveStateCommand(DriveState.TELEOP)
        );
    }

    public Command getAutonomousCommand() {
        Commands.runOnce(() -> robotDrive.setDriveState(DriveState.AUTON), robotDrive).schedule();

        return autoChooser.get();
    }

    public void getAutonomousExit() {
        robotDrive.setDriveState(DriveState.STOP);
    }

    private void configureStateTriggers() {
        new Trigger(DriverStation::isEnabled)
            .onTrue(
                Commands.runOnce(() -> robotDrive.resetModulesEncoders()));
    }

    private void configureButtonBindings() {
        if(useCompetitionBindings) {} 
        else {
            driverController.y().onTrue(Commands.runOnce(() -> robotDrive.resetGyro()));
    
            driverController.x()
                .onTrue(robotDrive.setDriveStateCommand(DriveState.DRIFT_TEST))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));

            // getPOV == -1 if nothing is pressed, so if it doesn't return that
            // then pov control is being used as its being pressed
            new Trigger(()-> driverController.getHID().getPOV() != -1)
                .onTrue(robotDrive.setDriveStateCommand(DriveState.POV_SNIPER))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));

            driverController.b()
                .onTrue(robotDrive.setDriveStateCommand(DriveState.LINEAR_TEST))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));

            driverController.a()
                .onTrue(robotDrive.setDriveStateCommandContinued(DriveState.AUTO_ALIGN))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));
        }
    }

}