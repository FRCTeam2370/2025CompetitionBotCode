// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.constant.DirectMethodHandleDesc;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ResetGyro;
import frc.robot.Commands.Drive.DriveRobotRelative;
import frc.robot.Commands.Drive.GoToSwervePose;
import frc.robot.Commands.Drive.TeleopSwerve;
import frc.robot.Commands.ElevatorCommands.ElevatorControl;
import frc.robot.Commands.ElevatorCommands.SetElevatorSpeed;
import frc.robot.Commands.ElevatorCommands.StowElevator;
import frc.robot.Commands.ManipulatorCommands.IntakeAlgae;
import frc.robot.Commands.ManipulatorCommands.IntakeCoral;
import frc.robot.Commands.ManipulatorCommands.RunAlgaeManipulator;
import frc.robot.Commands.ManipulatorCommands.RunManipulator;
import frc.robot.Commands.MechanismCommands.SetMechanismToPose;
import frc.robot.Commands.MechanismCommands.StowMechanismWithCoral;
import frc.robot.Commands.SwingArmCommands.SetSwingArm;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.ManipulatorSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.SwingArmSubsystem;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class RobotContainer {
  public static final CommandXboxController driver = new CommandXboxController(0);

  private final SwerveSubsystem mSwerve = new SwerveSubsystem();
  private final ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  private final ManipulatorSubsystem mManipulatorSubsystem = new ManipulatorSubsystem();
  private final SwingArmSubsystem mSwingArmSubsystem = new SwingArmSubsystem();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //Put all NamedCommands here
    NamedCommands.registerCommand("Test", new ResetGyro(mSwerve));
    
    configureBindings();
  }

  private void configureBindings() {
    mSwerve.setDefaultCommand(new TeleopSwerve(mSwerve, ()-> driver.getRawAxis(0), ()-> -driver.getRawAxis(1), ()-> driver.getRawAxis(4), ()-> false));

    //driver.b().toggleOnTrue(new GoToSwervePose(mSwerve, new Pose2d(new Translation2d(11.4, 7.7), Rotation2d.fromDegrees(90))));

    driver.start().onTrue(new ResetGyro(mSwerve));

    driver.leftBumper().toggleOnTrue(new IntakeCoral(mManipulatorSubsystem, 1));
    driver.rightBumper().whileTrue(new RunManipulator(mManipulatorSubsystem, -1));

    driver.leftTrigger().toggleOnTrue(new IntakeAlgae(mManipulatorSubsystem, 0.5));
    driver.rightTrigger().whileTrue(new RunAlgaeManipulator(mManipulatorSubsystem, -1));

    driver.leftStick().onTrue(new SetSwingArm(mSwingArmSubsystem, 0.165));//loading station
    //driver.x().onTrue(new SetSwingArm(mSwingArmSubsystem, 0));

    //driver.y().onTrue(new ElevatorControl(mElevatorSubsystem, 1.70));//L2
    // driver.y().onTrue(new ElevatorControl(mElevatorSubsystem, 5.1));//max height
    // driver.start().onTrue(new ElevatorControl(mElevatorSubsystem, 3));
    driver.b().onTrue(new StowMechanismWithCoral(mElevatorSubsystem, mSwingArmSubsystem));

    driver.y().onTrue(new SetMechanismToPose(1.55, 0.34, mSwingArmSubsystem, mElevatorSubsystem));//L2
    driver.a().onTrue(new SetMechanismToPose(2.609, 0.3177, mSwingArmSubsystem, mElevatorSubsystem));//L3
    driver.x().onTrue(new SetMechanismToPose(4.7, 0.32, mSwingArmSubsystem, mElevatorSubsystem));//L4

    driver.povDown().onTrue(new SetSwingArm(mSwingArmSubsystem, 0.2857));
    driver.povUp().onTrue(new SetSwingArm(mSwingArmSubsystem, 0.38));

    //driver.back().toggleOnTrue(new SetElevatorSpeed(mElevatorSubsystem));//this is for finding the kg for the elevator
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}