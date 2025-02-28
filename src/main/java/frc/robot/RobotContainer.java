// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.subsystems.vision.VisionConstants.Orientation;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPV;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser.SIDE;

import static frc.robot.subsystems.drive.DriveConstants.*;

import java.util.ArrayList;
import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utils.debugging.LoggedTunableNumber;

public class RobotContainer {
    // Define subsystems
    private final Drive robotDrive;
    private final Elevator elevator;
    private final Intake intake;
    private final Climb climb;
    
    // Define other utility classes
    private final AutonCommands autonCommands;
    private final TeleopCommands teleopCommands;
    
    private LoggedDashboardChooser<Command> autoChooser;
    
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    /* TODO: Set to true before competition
     please */

    private final boolean useCompetitionBindings = true;
    LED rightLED = new LED(9, 36);
    //LED leftLED = new LED(8, 36);

    // Anshul said to use this because he loves event loops
    private final EventLoop teleopLoop = new EventLoop();

    public RobotContainer() {
        led = new LED(LEDConstants.kLED);


        // If using AdvantageKit, perform mode-specific instantiation of subsystems.
        switch (Constants.kCurrentMode) {
            case REAL:
               robotDrive = new Drive( new Module[] {
                    new Module("FL", new ModuleIOKraken(kFrontLeftHardware )),
                    new Module("FR", new ModuleIOKraken(kFrontRightHardware)),
                    new Module("BL", new ModuleIOKraken(kBackLeftHardware  )),
                    new Module("BR", new ModuleIOKraken(kBackRightHardware ))
                }, new GyroIOPigeon2(), new Vision(new CameraIO[] {
                    new CameraIOPV(VisionConstants.kRightCamName, VisionConstants.kRightCamTransform, Orientation.BACK), 
                    new CameraIOPV(VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform, Orientation.BACK)
                }));

                elevator = new Elevator(
                    new ElevatorIOTalonFX(
                        Constants.kCanbusName, 
                        ElevatorConstants.kRoboElevatorHardware, 
                        ElevatorConstants.kMotorConfiguration, 
                        ElevatorConstants.kElevatorGains),
                    new MagneticSensorIO(){});
                    // new MagneticSensorIORev(ElevatorConstants.kSensorHardware));
            
                intake = new Intake(
                    new IntakeIOTalonFX(
                        IntakeConstants.kIntakeHardware,
                        IntakeConstants.kIntakeMotorConfiguration), 
                    new SensorIOLaserCAN(IntakeConstants.kSensorConfiguration),
                    // new SensorIO() {},
                    // new PivotIO(){});
                    new PivotIOTalonFX(
                        IntakeConstants.kPivotMotorHardware,
                        IntakeConstants.kPivotMotorConfiguration,
                        IntakeConstants.kPivotGains,
                        IntakeConstants.kStatusSignalUpdateFrequencyHz));

                // robotDrive = new Drive( new Module[] {
                //     new Module("FL", new ModuleIO() {}),
                //     new Module("FR", new ModuleIO() {}),
                //     new Module("BL", new ModuleIO() {}),
                //     new Module("BR", new ModuleIO() {})
                // }, new GyroIO() {}, new Vision(new CameraIO[] {
                //     new CameraIO() {}, new CameraIO() {}
                // }));

                // elevator = new Elevator(new ElevatorIO(){}, new MagneticSensorIO(){});
            
                // intake = new Intake(new IntakeIO(){}, new SensorIO(){}, new PivotIO(){});
            
                climb = new Climb(
                    new DutyCycleEncoderIORev(
                        ClimbConstants.kDutyCycleConfiguration),
                    new ClimbIOTalonFX(
                        ClimbConstants.kLeadMotorHardware, 
                        ClimbConstants.kLeadMotorConfiguration, 
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
                    new CameraIOPV(VisionConstants.kRightCamName, VisionConstants.kRightCamTransform, Orientation.BACK), 
                    new CameraIOPV(VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform, Orientation.BACK)
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
        teleopCommands = new TeleopCommands(elevator, intake, climb);
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
        // elevator.setDefaultCommand(Commands.run(() -> elevator.setGoal(ElevatorGoal.kStow), elevator));
        // intake.setDefaultCommand(
        //     Commands.run(
        //         () -> {
        //             intake.setPivotGoal(PivotGoal.kStow);
        //             intake.stop(true, false);
        //         }, 
        //         intake));

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

        new Trigger(DriverStation::isEnabled)
            .onTrue(
                Commands.runOnce(() -> 
                led.setGradientAnimation(
                    100, 
                    GradientType.kContinuous, 
                    Color.kLightBlue, 
                    Color.kMediumBlue, 
                    Color.kDarkBlue)));

        new Trigger(intake::detectedGamepiece)
        .whileTrue(
            Commands.runOnce(() -> 
                led.setSolidBlinkAnimation(
                0.5, 
                Color.kRed)))
        .whileFalse(
            Commands.runOnce(() -> 
                led.setGradientAnimation(
                    100,                     
                    GradientType.kContinuous, 
                    Color.kLightBlue, 
                    Color.kMediumBlue, 
                    Color.kDarkBlue)));
        
    }

    private Command rumbleCommand() {
        return Commands.startEnd(
            () -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 1.0), 
            () -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0));
    }

    private void configureButtonBindings() {
        HashMap<Trigger, Pair<ElevatorGoal, ElevatorGoal>> reefPositions = 
            new HashMap<Trigger, Pair<ElevatorGoal, ElevatorGoal>>();
        reefPositions.put(operatorController.y(), new Pair<>(ElevatorGoal.kL4Coral, ElevatorGoal.kL4Algae));
        reefPositions.put(operatorController.b(), new Pair<>(ElevatorGoal.kL3Coral, ElevatorGoal.kL3Algae));
        reefPositions.put(operatorController.a(), new Pair<>(ElevatorGoal.kL2Coral, ElevatorGoal.kL2Algae));
        reefPositions.put(operatorController.x(), new Pair<>(ElevatorGoal.kL1Coral, ElevatorGoal.kGroundAlgae));

        ArrayList<Trigger> positionButtons = new ArrayList<Trigger>();
        positionButtons.add(operatorController.y());
        positionButtons.add(operatorController.b());
        positionButtons.add(operatorController.a());
        positionButtons.add(operatorController.x());

        // Auto rumble if we are pressing intake button and we already have a gamepiece
        new Trigger(
            teleopLoop,
            intake::detectedGamepiece)
                .and(operatorController.leftBumper())
            .onTrue(
                rumbleCommand()
                    .withTimeout(0.5)
        );

        Trigger hasGamepieceTrigger = new Trigger(teleopLoop, intake::detectedGamepiece);
        Trigger elevatorAtGoalTrigger = new Trigger(teleopLoop, elevator::atGoal);
        Trigger pivotAtGoalTrigger = new Trigger(teleopLoop, intake::pivotAtGoal);
        Trigger coralSelectTrigger = operatorController.rightTrigger(0.5, teleopLoop);
        Trigger algaeSelectTrigger = operatorController.leftTrigger(0.5, teleopLoop);
        Trigger confirmScoreTrigger = operatorController.rightBumper(teleopLoop);

        if (useCompetitionBindings) {
            driverController.y().onTrue(Commands.runOnce(() -> robotDrive.resetGyro()));

            // getPOV == -1 if nothing is pressed, so if it doesn't return that
            // then pov control is being used as its being pressed
            new Trigger(()-> driverController.getHID().getPOV() != -1)
                .onTrue(robotDrive.setDriveStateCommandContinued(DriveState.POV_SNIPER))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));

            driverController.a()
                .onTrue(robotDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_REEF))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));

            driverController.b()
                .onTrue(robotDrive.setDriveStateCommandContinued(DriveState.PROCESSOR_HEADING_ALIGN))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));

            driverController.x()
                .onTrue(robotDrive.setDriveStateCommandContinued(DriveState.INTAKE_HEADING_ALIGN))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));

            driverController.leftBumper()
                .onTrue(GoalPoseChooser.setSideCommand(SIDE.LEFT));

            driverController.rightBumper()
                .onTrue(GoalPoseChooser.setSideCommand(SIDE.RIGHT));

            //TEMPORARY SCORE
            operatorController.rightBumper().and(coralSelectTrigger)
                .whileTrue(new InstantCommand(() -> intake.setRollerVoltage(6.0)))
                .whileFalse(new InstantCommand(() -> intake.setRollerVoltage(0.0)));

            //TEMPORARY SCORE
            operatorController.rightBumper().and(algaeSelectTrigger)
                .whileTrue(new InstantCommand(() -> intake.setRollerVoltage(3.0)))
                .whileFalse(new InstantCommand(() -> intake.setRollerVoltage(0.0)));

            // CORAL - INTAKE
            operatorController.leftBumper().and(coralSelectTrigger)
                .whileTrue(
                    teleopCommands.runElevatorAndHoldCommand(ElevatorGoal.kIntake)
                        .alongWith(
                            teleopCommands.runRollersAndStopCommand(RollerGoal.kIntakeCoral)
                                .onlyWhile(hasGamepieceTrigger.negate()))
                )
                .whileFalse(
                    teleopCommands.stopElevatorCommand()
                        .andThen(teleopCommands.stopRollersCommand())
                );

            // ALGAE - INTAKE
            operatorController.leftBumper().and(algaeSelectTrigger)
                .whileTrue(
                    teleopCommands.runAlgaeAndStopCommand(RollerGoal.kIntakeAlgae, PivotGoal.kIntakeGround)
                    .onlyWhile(hasGamepieceTrigger.negate())
                         .andThen(
                            teleopCommands.runElevatorAndHoldCommand(ElevatorGoal.kL1Coral)
                            // .alongWith(teleopCommands.runElevatorAndHoldCommand(ElevatorGoal.kL2Algae))
                        )
                )
                .whileFalse(
                    teleopCommands.stopRollersCommand()
                        .andThen(teleopCommands.runPivotAndStopCommand(PivotGoal.kStowPickup))
                        .alongWith(teleopCommands.runElevatorAndHoldCommand(ElevatorGoal.kStow))
                );

            // SCORE CORAL AND PICKUP ALGAE
            for (int i = 0; i < positionButtons.size(); i++) {
                Trigger button = positionButtons.get(i);
                Trigger algaePickup = positionButtons.get(i);
                Trigger algaeScore = positionButtons.get(i);

                // CORAL - SCORE
                button.and(coralSelectTrigger)
                    .whileTrue(
                        teleopCommands.runElevatorAndHoldCommand(reefPositions.get(button).getFirst())
                            // .onlyWhile(elevatorAtGoalTrigger.negate().debounce(0.5))
                            .beforeStarting(teleopCommands.selectGamepieceCommand(Gamepiece.kCoral))
                        .andThen(
                            teleopCommands.runRollersWhenConfirmed(RollerGoal.kScoreCoral, confirmScoreTrigger)
                        )   
                    )
                    .whileFalse(
                        teleopCommands.runElevatorAndHoldCommand(ElevatorGoal.kStow)
                    );

                if(i == 1 || i == 2){

                    // ALGAE - PICKUP
                    algaePickup.and(algaeSelectTrigger)
                        .whileTrue(
                        teleopCommands.runElevatorAndHoldCommand(reefPositions.get(button).getSecond())
                        .alongWith(
                                teleopCommands.runAlgaeAndStopCommand(RollerGoal.kIntakeAlgae, PivotGoal.kIntakeReef)
                                .onlyWhile(hasGamepieceTrigger.negate())
                               // .alongWith(teleopCommands.runElevatorAndHoldCommand(ElevatorGoal.kL2Algae))
                        )
                        )
                        .whileFalse(
                            teleopCommands.runElevatorAndHoldCommand(ElevatorGoal.kStow)
                            .alongWith(
                                    teleopCommands.runPivotAndStopCommand(PivotGoal.kStowPickup)
                                        // This "onlyWhile" is required so that this command composition ends
                                        // at some point and frees its resources back to the CommandScheduler
                                        .onlyWhile(pivotAtGoalTrigger.negate().debounce(0.5)))
                            .andThen(
                                teleopCommands.stopRollersCommand()
                            )

                        );
                }

                if(i == 0 || i == 3){
                    // ALGAE - SCORE
                    algaePickup.and(algaeSelectTrigger)
                        .whileTrue(
                        teleopCommands.runElevatorAndHoldCommand(reefPositions.get(button).getSecond())
                        .alongWith(
                                teleopCommands.runPivotAndStopCommand(PivotGoal.kProcessorScore)
                                    .onlyWhile(hasGamepieceTrigger.negate())
                                // .alongWith(teleopCommands.runElevatorAndHoldCommand(ElevatorGoal.kProcessor))
                        )
                        )
                        .whileFalse(
                            teleopCommands.runElevatorAndHoldCommand(ElevatorGoal.kStow)
                            .alongWith(
                                    teleopCommands.runPivotAndStopCommand(PivotGoal.kStowScore)
                                        // This "onlyWhile" is required so that this command composition ends
                                        // at some point and frees its resources back to the CommandScheduler
                                        .onlyWhile(pivotAtGoalTrigger.negate().debounce(0.5)))
                            .andThen(
                                teleopCommands.stopRollersCommand()
                            )

                        );
                }
            }

            // // ELEVATOR - UP
            // operatorController.povUp()
            //     .whileTrue(
            //         teleopCommands.runTunableVoltage()
            //     )
            //     .whileFalse(
            //         teleopCommands.stopElevatorCommand()
            //     );

            // // ELEVATOR - DOWN
            // operatorController.povDown()
            //     .whileTrue(
            //         teleopCommands.runElevatorVoltage(-3.0)
            //     )
            //     .whileFalse(
            //         teleopCommands.stopElevatorCommand()
            //     );

            // CLIMB - GRAB
            operatorController.povLeft()
                .whileTrue(
                    teleopCommands.runClimbVoltage(ClimbVoltageGoal.kGrab)
                )
                .whileFalse(
                    teleopCommands.stopClimbCommand()
                );
                
            // CLIMB - RELEASE
            operatorController.povRight()
                .whileTrue(
                    teleopCommands.runClimbVoltage(ClimbVoltageGoal.kRelease)
                )
                .whileFalse(
                    teleopCommands.stopClimbCommand()
                );

            // PIVOT - OUT
            // operatorController.povLeft()
            //     .whileTrue(
            //         teleopCommands.runPivotAndRollersVoltage(3.0, -3.0, confirmScoreTrigger)
            //     )
            //     .whileFalse(
            //         teleopCommands.stopRollersAndPivotCommand()
            //     );
                
            // // PIVOT - IN
            // operatorController.povRight()
            //     .whileTrue(
            //         teleopCommands.runPivotAndRollersVoltage(-3.0, -3.0, confirmScoreTrigger)
            //     )
            //     .whileFalse(
            //         teleopCommands.stopRollersAndPivotCommand()
            //     );

            operatorController.rightStick()
                .onTrue(Commands.runOnce(() -> elevator.resetPosition(), elevator));
        } 
        else {
            driverController.y().onTrue(Commands.runOnce(() -> robotDrive.resetGyro()));
    
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
                .onTrue(robotDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_NET))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));

            operatorController.povLeft()
                .whileTrue(
                    Commands.startEnd(
                        () -> climb.setGoalVoltage(ClimbVoltageGoal.kGrab), 
                        () -> climb.stop(), 
                        climb));

            operatorController.povRight()
                .whileTrue(
                    Commands.startEnd(
                        () -> climb.setGoalVoltage(ClimbVoltageGoal.kRelease), 
                        () -> climb.stop(), 
                        climb));

            operatorController.a()
                .whileTrue(
                    Commands.startEnd(
                        () -> climb.setGoalVoltage(ClimbVoltageGoal.custom), 
                        () -> climb.stop(), 
                        climb));

            operatorController.povUp()
                .whileTrue(
                    Commands.startEnd(
                        () -> elevator.setVoltage(3.0), 
                        () -> elevator.stop(), 
                        elevator));

            operatorController.povDown()
                .whileTrue(
                    Commands.startEnd(
                        () -> elevator.setVoltage(0.45), 
                        () -> elevator.stop(), 
                        elevator));

            operatorController.y()
                .whileTrue(
                    Commands.startEnd(
                        () -> elevator.setGoal(ElevatorGoal.kL2Coral), 
                        () -> elevator.stop(), 
                        elevator));       
        }
    }

    public EventLoop getTeleopEventLoop() {
        return teleopLoop;
    }

    public void updateVisualizers() {
        // Add a fudge factor to make the algae picker visualizer line up with the 
        // elevator better
        intake.setVisualizerVerticalPosition(elevator.getPositionMeters() + 0.38);
    }
}