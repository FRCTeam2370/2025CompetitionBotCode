// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ManipulatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ManipulatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpitPeice extends Command {
  private ManipulatorSubsystem mManipulatorSubsystem;
  private double speed;
  boolean isFinished = false;
  double counter;
  /** Creates a new SpitPeice. */
  public SpitPeice(double speed, ManipulatorSubsystem mManipulatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mManipulatorSubsystem = mManipulatorSubsystem;
    this.speed = speed;
    addRequirements(mManipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Started Spit Piece", true);
    isFinished = false;
    counter = 0;
  }


  @Override
  public void execute() {
    if(!ManipulatorSubsystem.hasCoral()){
      if(counter < 10){
        ManipulatorSubsystem.runManipulator(speed);
        counter++;
      }else{
        isFinished = true;
      }
    }else{
      ManipulatorSubsystem.runManipulator(speed);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ManipulatorSubsystem.runManipulator(0);
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
    
  }
}
