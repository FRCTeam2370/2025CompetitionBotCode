// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ManipulatorCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwingArmConstants;
import frc.robot.Subsystems.ManipulatorSubsystem;
import frc.robot.Subsystems.SwingArmSubsystem;
import frc.robot.Subsystems.ManipulatorSubsystem.AlgaeState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgae extends Command {
  private ManipulatorSubsystem mManipulatorSubsystem;
  private double speed;
  private Timer timer = new Timer();
  /** Creates a new RunAlgaeManipulator. */
  public IntakeAlgae(ManipulatorSubsystem mManipulatorSubsystem, double speed) {
    this.mManipulatorSubsystem = mManipulatorSubsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mManipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ManipulatorSubsystem.hasAlgae = false;
    timer.reset();
    timer.start();
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
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(ManipulatorSubsystem.algaeMotor.getOutputCurrent() >= 20 && timer.get() >= 0.1){//55
      ManipulatorSubsystem.hasAlgae = true;
      if(SwingArmSubsystem.getArmRotations() > 0){
        ManipulatorSubsystem.algaeState = AlgaeState.RIGHT;
      }else if(SwingArmSubsystem.getArmRotations() <= 0){
        ManipulatorSubsystem.algaeState = AlgaeState.LEFT;
      }
      return true;
    }else{
      return false;
    }
  }
}
