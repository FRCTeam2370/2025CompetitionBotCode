// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorSpeed extends Command {
  ElevatorSubsystem mElevatorSubsystem;
  double speed;
  /** Creates a new SetElevatorSpeed. */
  public SetElevatorSpeed(ElevatorSubsystem mElevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mElevatorSubsystem = mElevatorSubsystem;
    addRequirements(mElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.driver.a().getAsBoolean() == true){
      speed += 0.01;
    }else if(RobotContainer.driver.y().getAsBoolean() == true){
      speed -= 0.01;
    }
    ElevatorSubsystem.setElevator(speed);
    SmartDashboard.putNumber("Elevatorpercent", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ElevatorSubsystem.setElevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
