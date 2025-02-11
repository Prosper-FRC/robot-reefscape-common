// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

/** Constants for a Pivot (Single jointed arm) */
public class ClimbConstants {
    public record ClimbMotorConfiguration(
        boolean kEnableStatorCurrentLimit,
        boolean kEnableSupplyCurrentLimit,
        double kStatorCurrentLimitAmps,
        double kSupplyCurrentLimitAmps,
        NeutralModeValue kNeutralMode,
        boolean kInverted,
        double kPeakForwardVoltage,
        double kPeakReverseVoltage
    ) {}

    public record ClimbHardware(
        int motorId,
        Rotation2d absoluteEncoderOffset,
        double gearing,
        Rotation2d minPosisiton,
        Rotation2d maxPosisition
    ) {}

    public record SimulationConfiguration(
        DCMotor motorType,
        double measurementStdDevs
    ) {}

    
    public static final int kAbsoluteEncoderPort = 0;
    public static final Rotation2d kAbsoluteOffset = Rotation2d.fromRotations(0);

    public static final Rotation2d kMaxPosistionAngle = Rotation2d.fromRotations(0.5);
    public static final Rotation2d kMinPosistionAngle = Rotation2d.fromRotations(0);

    public static final double kClimbGearing = 125.0;

    public static final double kStatusSignalUpdateFrequencyHz = 100.0;
  
    public static final ClimbHardware kFollowerClimbHardwareConfiguration = new ClimbHardware(
        0, 
        kAbsoluteOffset,
        kClimbGearing, 
        kMinPosistionAngle, 
        kMaxPosistionAngle);

    public static final ClimbMotorConfiguration kLeadMotorConfiguration = new ClimbMotorConfiguration(
        true, 
        true, 
        60.0, 
        50.0, 
        NeutralModeValue.Brake, 
        false, 
        12.0, 
        -12.0);

    public static final ClimbHardware kLeadClimbHardwareConfiguration = new ClimbHardware(
        0, 
        kAbsoluteOffset,
        kClimbGearing, 
        kMinPosistionAngle, 
        kMaxPosistionAngle);

    public static final ClimbMotorConfiguration kFollowerMotorConfiguration = new ClimbMotorConfiguration(
        true, 
        true, 
        60.0, 
        50.0, 
        NeutralModeValue.Brake, 
        true, 
        12.0, 
        -12.0);
    
    public static final SimulationConfiguration kSimulationConfiguration = new SimulationConfiguration(
        DCMotor.getKrakenX60(1), 
        0.0002);
}
