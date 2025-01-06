// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveDrive extends SubsystemBase {
  private final SwerveModule frontLeftModule = new SwerveModule(
      ModuleConstants.fLDriveMotorID,
      ModuleConstants.fLTurnMotorID,
      ModuleConstants.fLAbsEncoderOffset);

  private final SwerveModule frontRightModule = new SwerveModule(
      ModuleConstants.bRDriveMotorID,
      ModuleConstants.bRTurnMotorID,
      ModuleConstants.bRAbsEncoderOffset);

  private final SwerveModule backLeftModule = new SwerveModule(
      ModuleConstants.bLDriveMotorID,
      ModuleConstants.bLTurnMotorID,
      ModuleConstants.bLAbsEncoderOffset);

  private final SwerveModule backRightModule = new SwerveModule(
      ModuleConstants.bRDriveMotorID,
      ModuleConstants.bRTurnMotorID,
      ModuleConstants.bRAbsEncoderOffset);

  private Pigeon2 gyro = new Pigeon2(1);
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
