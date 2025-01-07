// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;

/** An example command that uses an example subsystem. */
public class DriveJoystick extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //Getting the x and y axis of the joystick at the end of this file
  private Translation2d translation;

  //Geting the joystick for rotating the robot
  private double rotation;
  
  //Whether or not to use the gyro scope
  private boolean fieldRelative;
  
  private SwerveDrive swerve;
  private CommandXboxController driver;

  //Giving a rate to which the speed should increase in mps
  private SlewRateLimiter yLim = new SlewRateLimiter(3);
  private SlewRateLimiter xLim = new SlewRateLimiter(2);
  private SlewRateLimiter rotLim = new SlewRateLimiter(1);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveJoystick(SwerveDrive swerve, CommandXboxController driver, boolean fieldRelative) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);

    this.driver = driver;
    this.fieldRelative = fieldRelative;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Gets the values of the selected joysticks and axis's
    double yAxis = -driver.getLeftY();
    double xAxis = -driver.getLeftX();
    double rotAxis = -driver.getRightX();

    //If the joystick is less than the deadband set to 0 otherwise run the joystick at the value (Prevents Stick drift)
    yAxis = (Math.abs(yAxis) < SwerveConstants.deadband ? 0 : yAxis * 0.6);
    xAxis = (Math.abs(xAxis) < SwerveConstants.deadband ? 0 : xAxis * 0.6);
    rotAxis = (Math.abs(rotAxis) < SwerveConstants.deadband ? 0 : (rotAxis * 10));

    translation = new Translation2d(yAxis, xAxis).times(SwerveConstants.kMaxSpeedMPS);
    rotation = rotAxis * SwerveConstants.kMaxAngularSpeed;
    
    //Drives the robot based on the values of the joystick output
    swerve.drive(translation, rotAxis, fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}