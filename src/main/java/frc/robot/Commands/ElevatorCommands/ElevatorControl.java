// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorCommands;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorControl extends Command {
  ElevatorSubsystem mElevatorSubsystem;
  double elevatorPos;
  /** Creates a new ElevatorControl. */
  public ElevatorControl(ElevatorSubsystem mElevatorSubsystem, double elevatorPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mElevatorSubsystem = mElevatorSubsystem;
    this.elevatorPos = elevatorPos;
    addRequirements(mElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ElevatorSubsystem.mElevatorState = ElevatorState.Moving;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ElevatorSubsystem.setElevatorPosWithMagic(elevatorPos);
    //ElevatorSubsystem.setElevatorPos(elevatorPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ElevatorSubsystem.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(ElevatorSubsystem.getElevatorShaftRots() > elevatorPos * 0.95 && ElevatorSubsystem.getElevatorShaftRots() < elevatorPos * 1.05){
    //   ElevatorSubsystem.mElevatorState = ElevatorState.HoldingPosition;
    //   return true;
    // }else{
    //   return false;
    // }
    return false;
  }
}
