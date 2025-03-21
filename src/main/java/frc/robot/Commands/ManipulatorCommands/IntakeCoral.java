// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ManipulatorCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ManipulatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private ManipulatorSubsystem mManipulatorSubsystem;
  private double speed;
  private boolean isFinished = false;
  private Timer timer = new Timer();
  /** Creates a new RunManipulator. */
  public IntakeCoral(LEDSubsystem mLedSubsystem, ManipulatorSubsystem mManipulatorSubsystem, double speed) {
    this.mManipulatorSubsystem = mManipulatorSubsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mManipulatorSubsystem, mLedSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ManipulatorSubsystem.runManipulator(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ManipulatorSubsystem.runManipulator(0);
    //LEDSubsystem.obtainedPieceLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(ManipulatorSubsystem.hasCoral()){
      return true;
    }else{
      return false;
    }
  }
}
