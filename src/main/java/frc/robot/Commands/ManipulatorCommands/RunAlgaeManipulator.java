// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ManipulatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunAlgaeManipulator extends Command {
  private ManipulatorSubsystem mManipulatorSubsystem;
  private double speed;
  /** Creates a new RunAlgaeManipulator. */
  public RunAlgaeManipulator(ManipulatorSubsystem mManipulatorSubsystem, double speed) {
    this.mManipulatorSubsystem = mManipulatorSubsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mManipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(speed < 0){
      ManipulatorSubsystem.hasAlgae = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ManipulatorSubsystem.runAlgaeIntake(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ManipulatorSubsystem.runAlgaeIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
