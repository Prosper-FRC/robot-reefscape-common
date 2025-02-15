// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.elevator.MagneticSensorIO;
import frc.robot.subsystems.elevator.MagneticSensorIORev;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.Intake.RollerGoal;
import frc.robot.subsystems.intake.Intake.Gamepiece;
import frc.robot.subsystems.intake.PivotIO;
import frc.robot.subsystems.intake.PivotIOSim;
import frc.robot.subsystems.intake.PivotIOTalonFX;
import frc.robot.subsystems.intake.SensorIO;
import frc.robot.subsystems.intake.SensorIOLaserCAN;
import frc.robot.subsystems.intake.Intake.PivotGoal;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.climb.DutyCycleEncoderIO;
import frc.robot.subsystems.climb.DutyCycleEncoderIORev;
import frc.robot.subsystems.climb.Climb.ClimbVoltageGoal;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOKraken;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPV;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.Drive.DriveState;

import static frc.robot.subsystems.drive.DriveConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
    // Define subsystems
    private final Drive robotDrive;
    private final Elevator elevator;
    private final Intake intake;
    private final Climb climb;
    
    // Define other utility classes
    private final AutonCommands autonCommands;
    private final TeleopCommands telopCommands;
    
    private LoggedDashboardChooser<Command> autoChooser;
    
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    /* TODO: Set to true before competition please */
    private final boolean useCompetitionBindings = true;

    // Anshul said to use this because he loves event loops
    private final EventLoop teleopLoop = new EventLoop();

    public RobotContainer() {

        // If using AdvantageKit, perform mode-specific instantiation of subsystems.
        switch (Constants.kCurrentMode) {
            case REAL:
               robotDrive = new Drive( new Module[] {
                    new Module("FL", new ModuleIOKraken(kFrontLeftHardware )),
                    new Module("FR", new ModuleIOKraken(kFrontRightHardware)),
                    new Module("BL", new ModuleIOKraken(kBackLeftHardware  )),
                    new Module("BR", new ModuleIOKraken(kBackRightHardware ))
                }, new GyroIOPigeon2(), new Vision(new CameraIO[] {
                    new CameraIOPV(
                        VisionConstants.kRightCamName, 
                        VisionConstants.kRightCamTransform, 
                        VisionConstants.kRightCamOrientation), 
                    new CameraIOPV(
                        VisionConstants.kLeftCamName, 
                        VisionConstants.kLeftCamTransform, 
                        VisionConstants.kLeftCamOrientation)
                }));

                elevator = new Elevator(
                    new ElevatorIOTalonFX(
                        Constants.kCanbusName, 
                        ElevatorConstants.kRoboElevatorHardware, 
                        ElevatorConstants.kMotorConfiguration, 
                        ElevatorConstants.kElevatorGains),
                    new MagneticSensorIORev(ElevatorConstants.kSensorHardware));
            
                intake = new Intake(
                    new IntakeIOTalonFX(
                        IntakeConstants.kIntakeHardware,
                        IntakeConstants.kIntakeMotorConfiguration), 
                    new SensorIOLaserCAN(IntakeConstants.kSensorConfiguration),
                    new PivotIOTalonFX(
                        IntakeConstants.kPivotMotorHardware,
                        IntakeConstants.kPivotMotorConfiguration,
                        IntakeConstants.kPivotGains,
                        IntakeConstants.kStatusSignalUpdateFrequencyHz));
            
                climb = new Climb(
                    new DutyCycleEncoderIORev(
                        ClimbConstants.kDutyCycleConfiguration),
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
               robotDrive = new Drive( new Module[] {
                    new Module("FL", new ModuleIOSim()),
                    new Module("FR", new ModuleIOSim()),
                    new Module("BL", new ModuleIOSim()),
                    new Module("BR", new ModuleIOSim())
                }, new GyroIO() {}, new Vision(new CameraIO[] {
                    new CameraIOPV(
                        VisionConstants.kRightCamName, 
                        VisionConstants.kRightCamTransform, 
                        VisionConstants.kRightCamOrientation), 
                    new CameraIOPV(
                        VisionConstants.kLeftCamName, 
                        VisionConstants.kLeftCamTransform, 
                        VisionConstants.kLeftCamOrientation)
                }));
            
                elevator = new Elevator(
                    new ElevatorIOSim(ElevatorConstants.kRoboElevatorHardware,
                    ElevatorConstants.kSimulationConfiguration,
                    ElevatorConstants.kElevatorGains,
                    ElevatorConstants.kMinPositionMeters,
                    ElevatorConstants.kMaxPositionMeters,
                    0.02),
                    new MagneticSensorIO(){});
                    
                intake = new Intake(
                    new IntakeIOSim(
                        IntakeConstants.kIntakeHardware, 
                        IntakeConstants.kIntakeSimulationConfiguration, 
                        0.02), 
                        new SensorIO(){},
                        new PivotIOSim(
                            0.02,
                            IntakeConstants.kPivotMotorHardware,
                            IntakeConstants.kPivotSimulationConfiguration,
                            IntakeConstants.kPivotGains));

                climb = new Climb(
                    new DutyCycleEncoderIO(){},
                    new ClimbIOSim(
                        0.02,
                        ClimbConstants.kLeadMotorHardware,
                        ClimbConstants.kSimulationConfiguration,
                        ClimbConstants.kMotorGains));
                break;
            default:
               robotDrive = new Drive( new Module[] {
                    new Module("FL", new ModuleIO() {}),
                    new Module("FR", new ModuleIO() {}),
                    new Module("BL", new ModuleIO() {}),
                    new Module("BR", new ModuleIO() {})
                }, new GyroIO() {}, new Vision(new CameraIO[] {
                    new CameraIO() {}, new CameraIO() {}
                }));

                elevator = new Elevator(new ElevatorIO(){}, new MagneticSensorIO(){});
            
                intake = new Intake(new IntakeIO(){}, new SensorIO(){}, new PivotIO(){});
            
                climb = new Climb(
                    new DutyCycleEncoderIO(){}, 
                    new ClimbIO[]{new ClimbIO(){}});
                break;
        }

        // Instantiate subsystems that don't care about mode, or are non-AdvantageKit enabled.
        // ex: LEDs = new LEDSubsystem();

        // Instantiate your TeleopCommands and AutonCommands classes
        telopCommands = new TeleopCommands(elevator, intake, climb);
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

        robotDrive.setDefaultCommand(Commands.run(() -> robotDrive.setDriveState(DriveState.TELEOP), robotDrive));

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

    /* Commands to schedule on telop start-up */
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
        /* Due to roborio start up times sometimes modules aren't reset properly, this accounts for that */
        new Trigger(DriverStation::isEnabled)
            .onTrue(
                Commands.runOnce(() -> robotDrive.resetModulesEncoders()));
    }

    private void configureButtonBindings() {
        Command startRumbleCommand = Commands.runOnce(() -> operatorController.setRumble(RumbleType.kBothRumble, 0.75));
        Command stopRumbleCommand = Commands.runOnce(() -> operatorController.setRumble(RumbleType.kBothRumble, 0.0));

        Trigger autoSelectCoral = operatorController.rightTrigger(0.5, teleopLoop)
            .onTrue(Commands.runOnce(() -> intake.selectGamepiece(Gamepiece.kCoral)));
        Trigger autoSelectAlgae = operatorController.leftTrigger(0.5, teleopLoop)
            .onTrue(Commands.runOnce(() -> intake.selectGamepiece(Gamepiece.kAlgae)));
        Trigger autoRumble = new Trigger(teleopLoop, intake::detectedGamepiece).debounce(0.4)
            .and(operatorController.leftBumper())
            .whileTrue(startRumbleCommand)
            .whileFalse(stopRumbleCommand);

        Trigger hasGamepieceTrigger = new Trigger(teleopLoop, intake::detectedGamepiece).debounce(0.4);
        Trigger elevatorAtGoalTrigger = new Trigger(teleopLoop, elevator::atGoal);
        Trigger coralSelectTrigger = operatorController.rightTrigger(0.5, teleopLoop);
        Trigger algaeSelectTrigger = operatorController.leftTrigger(0.5, teleopLoop);
        Trigger confirmScoreTrigger = operatorController.rightBumper(teleopLoop);

        if (useCompetitionBindings) {
            /* Coral bindings */
            operatorController.leftBumper().and(coralSelectTrigger)
                .whileTrue(
                    Commands.run(() -> {
                        intake.setRollerGoal(RollerGoal.kIntakeCoral);
                    }, 
                    intake)
                    .until(hasGamepieceTrigger)
                );

            operatorController.y().and(coralSelectTrigger)
                .whileTrue(
                    Commands.runEnd(
                        () -> elevator.setGoal(ElevatorGoal.kL4Coral),
                        () -> elevator.setPosition(elevator.getPositionMeters()),
                        elevator)
                    .until(elevatorAtGoalTrigger)
                    .andThen(
                        Commands.run(() -> {
                            if (confirmScoreTrigger.getAsBoolean()) {
                                intake.setRollerGoal(RollerGoal.kScoreCoral);
                            } else {
                                intake.stop(true, false);
                            }
                        }, 
                        intake)
                    )
                )
                .whileFalse(
                    Commands.runOnce(
                        () -> elevator.stop(), 
                        elevator)
                    .alongWith(
                        Commands.runOnce(
                            () -> intake.stop(true, false), 
                            intake)
                    )
                );
        } 
        else {
            driverController.y().onTrue(Commands.runOnce(() -> robotDrive.setPose(new Pose2d(0.0, 0.0, Rotation2d.k180deg))));
    
            driverController.x()
                .onTrue(robotDrive.setDriveStateCommandContinued(DriveState.TELEOP_SNIPER))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));

            // getPOV == -1 if nothing is pressed, so if it doesn't return that
            // then pov control is being used as its being pressed
            new Trigger(()-> driverController.getHID().getPOV() != -1)
                .onTrue(robotDrive.setDriveStateCommandContinued(DriveState.POV_SNIPER))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));

            driverController.b()
                .onTrue(robotDrive.setDriveStateCommandContinued(DriveState.LINEAR_TEST))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));

            driverController.a()
                .onTrue(robotDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_POSE))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));
        }
    }

    public EventLoop getTeleopEventLoop() {
        return teleopLoop;
    }
}