// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.MechanismCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.ManipulatorCommands.IntakeCoral;
import frc.robot.Commands.ManipulatorCommands.IntakeCoralBetter;
import frc.robot.Commands.SwingArmCommands.SetSwingArm;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ManipulatorSubsystem;
import frc.robot.Subsystems.SwingArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MechanismToLoading extends ParallelCommandGroup {
  /** Creates a new MechanismToLoading. */
  public MechanismToLoading(ManipulatorSubsystem manipulatorSubsystem, SwingArmSubsystem mSwingArmSubsystem, LEDSubsystem mLedSubsystem, ElevatorSubsystem mElevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetSwingArm(mSwingArmSubsystem,  -0.124).alongWith(new IntakeCoralBetter(manipulatorSubsystem, mLedSubsystem)));// -0.125
  }
}
