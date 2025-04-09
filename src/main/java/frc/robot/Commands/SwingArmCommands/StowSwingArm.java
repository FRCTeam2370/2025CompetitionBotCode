// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SwingArmCommands;

import edu.wpi.first.util.MsvcRuntimeException;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ManipulatorSubsystem;
import frc.robot.Subsystems.SwingArmSubsystem;
import frc.robot.Subsystems.ManipulatorSubsystem.AlgaeState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StowSwingArm extends Command {
  SwingArmSubsystem mSwingArmSubsystem;
  private double pos = 0;
  /** Creates a new StowSwingArm. */
  public StowSwingArm(SwingArmSubsystem mSwingArmSubsystem, ManipulatorSubsystem mManipulatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mSwingArmSubsystem = mSwingArmSubsystem;
    addRequirements(mSwingArmSubsystem, mManipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ManipulatorSubsystem.algaeState == AlgaeState.RIGHT){
      pos = 0.3;
    }else if(ManipulatorSubsystem.algaeState == AlgaeState.LEFT){
      pos = -0.3;
    }else{
      pos = 0;
    }

    SwingArmSubsystem.setSwingArmPos(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(ManipulatorSubsystem.hasAlgae){
    //   if(SwingArmSubsystem.getArmRotations() < pos + 0.5 && SwingArmSubsystem.getArmRotations() > pos - 0.5){
    //     return true;
    //   }else{
    //     return false;
    //   }
    //  }//else{
    // //   if(SwingArmSubsystem.getArmRotations() < 0.05 && SwingArmSubsystem.getArmRotations() > -0.05){
    // //     return true;
    // //   }else{
    // //     return false;
    // //   }
    // // }
    if(SwingArmSubsystem.getArmRotations() < pos + 0.5 && SwingArmSubsystem.getArmRotations() > pos - 0.5){
        return true;
      }else{
        return false;
      }
  }
}
