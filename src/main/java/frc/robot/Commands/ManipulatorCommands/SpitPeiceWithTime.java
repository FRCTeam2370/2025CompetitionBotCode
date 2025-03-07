// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Subsystems.ManipulatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpitPeiceWithTime extends ParallelCommandGroup {
  /** Creates a new SpitPeiceWithTime. */
  public SpitPeiceWithTime(ManipulatorSubsystem manipulatorSubsystem, double waitTime) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunManipulator(manipulatorSubsystem, -0.5).alongWith(new WaitCommand(waitTime)));
  }
}
