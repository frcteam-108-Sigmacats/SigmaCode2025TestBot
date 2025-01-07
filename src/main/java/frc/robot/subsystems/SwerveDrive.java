// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;

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

  private SwerveDrivePoseEstimator swervePoseEstimator;

  private SwerveModule[] modules = {frontLeftModule, frontRightModule, backLeftModule, backRightModule};
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    gyro.getConfigurator().apply(new Pigeon2Configuration());

    gyro.clearStickyFault_BootDuringEnable();

    swervePoseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, null, null, new Pose2d());
  }

  //Gets the angle of the robots direction
    public Rotation2d getHeading(){
      return Rotation2d.fromDegrees(-Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360));
    }

  public Pose2d getPose(){
    return swervePoseEstimator.getEstimatedPosition();
  }

  public void zeroHeading(){
    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //Sets the speed and direction of each module and adjusts based on if we want to drive field relative or not
    public void drive(Translation2d translation, double rotation, boolean fieldRelative){
      SwerveModuleState[] swerveModuleStates =
          SwerveConstants.swerveKinematics.toSwerveModuleStates(
              fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                  translation.getX(), 
                                  translation.getY(), 
                                  rotation, 
                                  getYaw()
                              )
                              : new ChassisSpeeds(
                                  translation.getX(), 
                                  translation.getY(), 
                                  rotation)
                              );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kMaxSpeedMPS);
        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        backLeftModule.setDesiredState(swerveModuleStates[2]);
        backRightModule.setDesiredState(swerveModuleStates[3]);
    }

    //Gets the 4 modules current state
    public SwerveModuleState[] getModuleStates(){
      SwerveModuleState[] states = new SwerveModuleState[4];
      for(int i = 0; i < states.length; i++){
        states[i] = modules[i].getState();
      }
      return states;
    }

    //Gets the modules position (How much did the robot drive forward or backwards and left or right in meters and the direction the wheels are facing)
    public SwerveModulePosition[] getModulePosition(){
      SwerveModulePosition[] positions = new SwerveModulePosition[4];
      for(int i = 0; i < positions.length; i++){
        positions[i] = modules[i].getPosition();
      }
      return positions;
    }

    //Resetting the Pose Estimator
      public void resetEstimator(Pose2d pose){
        swervePoseEstimator.resetPosition(getHeading(), getModulePosition(), pose);
      }

    //Sets the drive encoders to 0
    public void resetEncoders(){
      frontLeftModule.resetEncoders();
      frontRightModule.resetEncoders();
      backLeftModule.resetEncoders();
      backRightModule.resetEncoders();
    }

  //Same thing as getting the heading but is not restricted to 360 degrees
  public Rotation2d getYaw(){
    double yaw = gyro.getYaw().getValueAsDouble();
    return (SwerveConstants.gyroReversed) ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
  }
}
