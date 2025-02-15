package frc.robot.utils.debugging;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// import static frc.robot.subsystems.drive.DriveConstants.kDrivebaseRadiusMeters;
// import static frc.robot.subsystems.drive.DriveConstants.kRadiusMeters;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

// For rev logs extract using wpilib's data log tool:
// https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog-download.html
// For talon logs extract using phoenix tuner x:
// https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/tools/log-extractor.html
public class SysIDCharacterization {
    public static Command runFlywheelSysIDTests(Consumer<Double> voltageSetter, Subsystem subsystem) {
        SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Second),
                Units.Volts.of(4),
                Units.Seconds.of(15),
                (state) -> sysIDCTREStateLogger("SysID/Shooter", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> voltageSetter.accept(voltage.magnitude()), null, subsystem));

        return new SequentialCommandGroup(
            startCTRELoggingRoutine(),
            Commands.waitSeconds(10.0),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(10.0),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(10.0),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(10.0),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(10.0),
            stopCTRELoggingRoutine());
        // For rev logs extract using wpilib's data log tool:
        // https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog-download.html
        // For talon logs extract using phoenix tuner x:
        // https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/tools/log-extractor.html
    }

    public static Command runDriveSysIDTests(Consumer<Double> voltageSetter, Subsystem subsystem) {
        SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Second),
                Units.Volts.of(3),
                Units.Seconds.of(5),
                (state) -> sysIDCTREStateLogger("SysID/Drive", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> voltageSetter.accept(voltage.magnitude()), null, subsystem));

        return new SequentialCommandGroup(
            startCTRELoggingRoutine(),
            Commands.waitSeconds(3.0),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(3.0),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(3.0),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(3.0),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(3.0),
            stopCTRELoggingRoutine()
        );
    }

    private static Command startCTRELoggingRoutine() {
        return Commands.runOnce(() -> SignalLogger.start());
    }

    private static void sysIDCTREStateLogger(String key, String state) {
        SignalLogger.writeString(key, state);
    }

    private static Command stopCTRELoggingRoutine() {
        return Commands.runOnce(() -> SignalLogger.stop());
    }

    private static void sysIDREVStateLogger(String key, String state) {
        Logger.recordOutput(key, state);
    }
}
