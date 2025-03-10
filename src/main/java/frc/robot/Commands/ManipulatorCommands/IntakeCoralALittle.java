// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ManipulatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralALittle extends Command {
  ManipulatorSubsystem manipulatorSubsystem;
  double counter;
  boolean isFinished;
  /** Creates a new IntakeCoralALittle. */
  public IntakeCoralALittle(ManipulatorSubsystem manipulatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(counter < 10){
      ManipulatorSubsystem.runManipulator(1);
      counter ++;
    }else{
      ManipulatorSubsystem.runManipulator(0);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
