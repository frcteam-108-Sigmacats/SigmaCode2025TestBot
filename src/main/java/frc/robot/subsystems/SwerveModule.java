// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  private SparkMax driveMotor;
  private SparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private AbsoluteEncoder absEncoder;
  private AbsoluteEncoderConfig absEncoderConfigs = new AbsoluteEncoderConfig();

  private SparkClosedLoopController driveClosedLoopController;
  private SparkClosedLoopController turnClosedLoopController;

  private SparkBaseConfig driveConfig;
  private SparkBaseConfig turnConfig;

  private double absAngleOffset;

  private SwerveModuleState m_desiredState = new SwerveModuleState();

  

  /** Creates a new ExampleSubsystem. */
  public SwerveModule(int driveMotorID, int turnMotorID, double angleOffset) {
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);

    driveConfig.idleMode(ModuleConstants.driveIdleMode);
    turnConfig.idleMode(ModuleConstants.turnIdleMode);

    driveConfig.smartCurrentLimit(ModuleConstants.driveMotorCurrentLimit);
    turnConfig.smartCurrentLimit(ModuleConstants.turnMotorCurrentLimit);

    driveConfig.encoder.positionConversionFactor(ModuleConstants.drivingFactor);
    driveConfig.encoder.velocityConversionFactor(ModuleConstants.drivingFactor / 60.0);

    driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(ModuleConstants.driveP,ModuleConstants.driveI, ModuleConstants.driveD)
    .velocityFF(1 / ModuleConstants.kFreeSpeedRPM)
    .outputRange(-1.0, 1.0);

    turnConfig.idleMode(ModuleConstants.turnIdleMode);
    turnConfig.smartCurrentLimit(ModuleConstants.turnMotorCurrentLimit);

    turnConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.turningFactor)
    .velocityConversionFactor(ModuleConstants.turningFactor / 60.0)
    .inverted(true);

    turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(ModuleConstants.turnP, ModuleConstants.turnI, ModuleConstants.turnD)
    .outputRange(-1.0, 1.0)
    .positionWrappingEnabled(true)
    .positionWrappingInputRange(0, ModuleConstants.turningFactor);

    driveEncoder = driveMotor.getEncoder();
    absEncoder = turnMotor.getAbsoluteEncoder();

    driveClosedLoopController = driveMotor.getClosedLoopController();
    turnClosedLoopController = turnMotor.getClosedLoopController();

    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    absAngleOffset = angleOffset;

    m_desiredState.angle = new Rotation2d(absEncoder.getPosition());
    driveEncoder.setPosition(0);
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(driveEncoder.getVelocity(),
        new Rotation2d(absEncoder.getPosition() - absAngleOffset));
  }

  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        new Rotation2d(absEncoder.getPosition() - absAngleOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(absAngleOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(absEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    driveClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    turnClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
