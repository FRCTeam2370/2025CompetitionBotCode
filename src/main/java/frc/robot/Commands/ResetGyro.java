// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetGyro extends Command {
  SwerveSubsystem mSwerve;
  /** Creates a new ResetGyro. */
  public ResetGyro(SwerveSubsystem mSwerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mSwerve = mSwerve;
    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveSubsystem.resetGyro();
    if(SwerveSubsystem.isBlue()){
      mSwerve.resetOdometry(new Pose2d(SwerveSubsystem.poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(SwerveSubsystem.gyro.getYaw().getValueAsDouble() + 90)));
    }else{
      mSwerve.resetOdometry(new Pose2d(SwerveSubsystem.poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(SwerveSubsystem.gyro.getYaw().getValueAsDouble() + 270)));
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the commnd should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
