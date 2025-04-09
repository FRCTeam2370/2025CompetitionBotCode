// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SwingArmCommands;

import edu.wpi.first.util.MsvcRuntimeException;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Subsystems.ManipulatorSubsystem;
import frc.robot.Subsystems.SwingArmSubsystem;
import frc.robot.Subsystems.ManipulatorSubsystem.AlgaeState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetSwingArm extends Command {
  private SwingArmSubsystem mSwingArmSubsystem;
  private double pos;
  private double outPutPos;
  /** Creates a new SetSwingArm. */
  public SetSwingArm(SwingArmSubsystem mSwingArmSubsystem, double pos) {
    this.mSwingArmSubsystem = mSwingArmSubsystem;
    this.pos = pos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mSwingArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(ManipulatorSubsystem.algaeState == AlgaeState.RIGHT){
      outPutPos = Math.abs(pos);
    }else if(ManipulatorSubsystem.algaeState == AlgaeState.LEFT){
      outPutPos = -Math.abs(pos);
    }else if(ManipulatorSubsystem.algaeState == AlgaeState.NONE){
      outPutPos = pos;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwingArmSubsystem.setSwingArmPos(outPutPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
