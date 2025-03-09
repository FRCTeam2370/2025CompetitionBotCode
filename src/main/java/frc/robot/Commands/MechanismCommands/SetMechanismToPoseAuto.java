// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.MechanismCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ElevatorCommands.ElevatorAutoPose;
import frc.robot.Commands.ElevatorCommands.ElevatorControl;
import frc.robot.Commands.ManipulatorCommands.SpitPeice;
import frc.robot.Commands.SwingArmCommands.SetSwingArm;
import frc.robot.Commands.SwingArmCommands.SetSwingArmAuto;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.ManipulatorSubsystem;
import frc.robot.Subsystems.SwingArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetMechanismToPoseAuto extends SequentialCommandGroup {
  /** Creates a new SetMechanismToPose. */
  public SetMechanismToPoseAuto(double ElevatorPos, double SwingArmPos, SwingArmSubsystem mSwingArmSubsystem, ElevatorSubsystem mElevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands((new ElevatorAutoPose(mElevatorSubsystem, ElevatorPos).alongWith(new SetSwingArmAuto(mSwingArmSubsystem, SwingArmPos))));
  }
}
