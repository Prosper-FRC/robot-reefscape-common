package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

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

    // ROBOT SEPCIFIC
    public static final String kCanbusName = "drivebase";

    public static final double kFieldLengthMeters = 17.54;
    public static final double kFieldWidthMeters = 8.05;

    public static final int kAprilTagCount = 22;

    public static final double kLoopPeriod = 0.02;
}
