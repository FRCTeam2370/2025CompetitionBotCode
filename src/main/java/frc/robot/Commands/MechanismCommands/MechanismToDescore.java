// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.MechanismCommands;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.ElevatorCommands.ElevatorControl;
import frc.robot.Commands.ElevatorCommands.SetElevatorForDescore;
import frc.robot.Commands.ManipulatorCommands.IntakeAlgae;
import frc.robot.Commands.SwingArmCommands.SetSwingArmForDescore;
import frc.robot.Lib.Utils.SwervePOILogic;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ManipulatorSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.SwingArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MechanismToDescore extends ParallelCommandGroup {
  /** Creates a new MechanismToDescore. */
  public MechanismToDescore(ManipulatorSubsystem manipulatorSubsystem, ElevatorSubsystem mElevatorSubsystem, SwingArmSubsystem mSwingArmSubsystem, LEDSubsystem mLedSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new SetSwingArmForDescore(mSwingArmSubsystem).alongWith(new SetElevatorForDescore(mElevatorSubsystem))
          .alongWith(new IntakeAlgae(manipulatorSubsystem, 0.5)));
  }
}