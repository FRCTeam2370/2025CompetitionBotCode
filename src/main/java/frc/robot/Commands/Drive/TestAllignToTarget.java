// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAllignToTarget extends SequentialCommandGroup {
  
  /** Creates a new TestAllignToTarget. */
  public TestAllignToTarget(SwerveSubsystem mSwerveSubsystem, Pose2d pose, boolean left) {
    new Rotation2d();
    Pose2d rPose = pose;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(mSwerveSubsystem.PathfindToPose(()-> rPose).andThen(new AlignToTagWithTX(mSwerveSubsystem, left)));
  }
}
