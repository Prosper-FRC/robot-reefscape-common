// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;

/** Constants for a Pivot (Single jointed arm) */
public class ClimbConstants {
    /** Motor configuration constants */
    public record KrakenConfiguration(
        boolean kEnableStatorCurrentLimit,
        boolean kEnableSupplyCurrentLimit,
        double kStatorCurrentLimitAmps,
        double kSupplyCurrentLimitAmps,
        NeutralModeValue kNeutralMode,
        boolean kInverted,
        double kPeakForwardVoltage,
        double kPeakReverseVoltage
    ) {}

    /** Feedback, feedforward, and profile constraints */
    public record PivotGains(
        double kP,
        double kI,
        double kD,
        double kS,
        double kG,
        double kA,
        double kV,
        // Max velocity constraint for the Trapezoidal Motion Profile
        double kMaxVelocity,
        // Max acceleration constraint for the Trapezoidal Motion Profile
        double kMaxAcceleration
    ) {}

    // If using a CANivore, set this boolean to true and set the CANivore name to
    // what is is. Consult your electrical lead if you are unsure if your team
    // is using a CANivore. The name can be configured using Phoenix Tuner.
    public static final boolean kUseCANivore = false;
    public static final String kCANBusName = "drivebase";

    // The gearing between your motor shaft and output shaft, consult the
    // mechanical team for this value
    public static final double kGearRatio = 96.9 / 1.0;

    public static final int kLeaderMotorID = 41;
    public static final int kFollowerMotorID = 42;

    // If using a REV ThroughBore as your absolute encoder, set this value to true
    public static final boolean kUseThroughBore = false;
    // This is the digital input port of the RoboRIO that your through bore data pin plugs into
    public static final int kThroughBoreEncoderPort = 0;
    // If your absolute encoder is mounted anywhere except directly on to the shaft, you need to plug
    // in the gear ratio between the shaft that your absolute encoder is on, and shaft that your
    // mechanism is on
    public static final double kThroughBoreEncoderRatio = 1.0 / 1.0;

    // TODO Check if this is accurate
    // Whether or not the follower-motor will spin in the opposite direction of
    // the leader-motor. If the follower-motor is mounted facing the opposite
    // direction of the leader-motor, this value should be set to true.
    public static final boolean kFollowerMotorOpposeMasterDirection = true;

    /** The frequency that telemetry form the motor is pushed to the CANBus */
    public static final double kStatusSignalUpdateFrequencyHz = 100.0;

    // Soft positions limits to prevent the arm from breaking itself
    public static final Rotation2d kUpperPositionLimit = Rotation2d.fromDegrees(275.0);
    public static final Rotation2d kLowerPositionLimit = Rotation2d.fromDegrees(0.0);

    // NOTE The configuration only needs to be applied to the leader-motor. The
    // follower-motor MUST obey the configuration of the leader-motor.
    public static final KrakenConfiguration kMotorConfiguration =
        new KrakenConfiguration(true, true, 80.0, 30.0, NeutralModeValue.Brake, false, 12.0, -12.0);

    public static final PivotGains kPivotGains =
        new PivotGains(75.0, 0.0, 0.0, 0.24, 0.3, 0.0, 0.0, 10.0, 20.0);
}
