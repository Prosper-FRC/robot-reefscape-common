// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

import org.littletonrobotics.junction.LogFileUtil;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;

public class Robot extends LoggedRobot {
    private Command mAutonomousCommand;
    private Command mTeleopCommand;

    private RobotContainer mRobotContainer;
    

    // ==================== Robot Power On ====================
    @Override
    public void robotInit() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        switch (Constants.kCurrentMode) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                SignalLogger.setPath("/U/logs");
                // SignalLogger.enableAutoLogging(false);
                // SignalLogger.stop();
                break;
            case SIM:
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start();
        SignalLogger.start();

        mRobotContainer = new RobotContainer();

        PathfindingCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    // ==================== Disabled ====================
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    // ==================== Autonomous ====================
    @Override
    public void autonomousInit() {
        mAutonomousCommand = mRobotContainer.getAutonomousCommand();

        if (mAutonomousCommand != null) {
            mAutonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        mRobotContainer.getAutonomousExit();
    }

    // ==================== Teleop ====================
    @Override
    public void teleopInit() {
        if (mAutonomousCommand != null) {
            mAutonomousCommand.cancel();
        }
        mTeleopCommand = mRobotContainer.getTeleopCommand();
        if (mTeleopCommand != null) {
            mTeleopCommand.schedule();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    // ==================== Test ====================
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
