package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {

    // AdvantageKit modes
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
    // Set Tuning to true during development, false during competition
    public static final boolean kTuningMode = true;

    public static final Alliance kAlliance = DriverStation.getAlliance().isPresent() &&
        DriverStation.getAlliance().get() == Alliance.Red ? Alliance.Red : Alliance.Blue;

    // ROBOT SEPCIFIC
    public static final String kCanbusName = "drivebase";

    // TODO: FIND THIS OUT
    public static final double kFieldWidth = 16.54;
    public static final double kFieldLength = 16.54;

    public static final double kLoopPeriod = 0.02;
}
