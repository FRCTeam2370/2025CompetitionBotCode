// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SwingArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Subsystems.ManipulatorSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.SwingArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetSwingArmForDescore extends Command {
  /** Creates a new SetSwingArmForDescore. */
  public SetSwingArmForDescore(SwingArmSubsystem mArmSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwingArmSubsystem.setSwingArmPos(SwerveSubsystem.findBestDescore().getFirst());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(ManipulatorSubsystem.hasAlgae){
      return true;
    }else{
      return false;
    }
  }
}
