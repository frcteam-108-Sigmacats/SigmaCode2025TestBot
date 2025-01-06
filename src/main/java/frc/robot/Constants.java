// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final class ModuleConstants{
    public static final double kFreeSpeedRPM = 5676;
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRPM / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;


    public static final int fLDriveMotorID = 1;
    public static final int fLTurnMotorID = 2;
    public static final double fLAbsEncoderOffset = -Math.PI / 2;

    public static final int fRDriveMotorID = 3;
    public static final int fRTurnMotorID = 4;
    public static final double fRAbsEncoderOffset = 0;

    public static final int bLDriveMotorID = 5;
    public static final int bLTurnMotorID = 6;
    public static final double bLAbsEncoderOffset = Math.PI;

    public static final int bRDriveMotorID = 7;
    public static final int bRTurnMotorID = 8;
    public static final double bRAbsEncoderOffset = Math.PI / 2;

    public static final int driveMotorCurrentLimit = 50;
    public static final int turnMotorCurrentLimit = 20;

    public static final IdleMode driveIdleMode = IdleMode.kBrake;
    public static final IdleMode turnIdleMode = IdleMode.kCoast;

    public static final double driveP = 0.04;
    public static final double driveI = 0.0;
    public static final double driveD = 0.0;

    public static final double turnP = 1.0;
    public static final double turnI = 0.0;
    public static final double turnD = 0.0;

    public static final double drivingFactor = kWheelDiameterMeters * Math.PI
    / kDrivingMotorReduction;
    public static final double turningFactor = 2 * Math.PI;
    double drivingVelocityFeedForward = 1 / kDriveWheelFreeSpeedRps;
  }
}
