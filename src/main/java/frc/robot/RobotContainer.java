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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ResetGyro;
import frc.robot.Commands.ClimberCommands.ControlClimberManual;
import frc.robot.Commands.Drive.DriveRobotRelative;
import frc.robot.Commands.Drive.GoToSwervePose;
import frc.robot.Commands.Drive.TeleopSwerve;
import frc.robot.Commands.ElevatorCommands.ElevatorControl;
import frc.robot.Commands.ElevatorCommands.SetElevatorSpeed;
import frc.robot.Commands.ElevatorCommands.StowElevator;
import frc.robot.Commands.ManipulatorCommands.IntakeAlgae;
import frc.robot.Commands.ManipulatorCommands.IntakeCoral;
import frc.robot.Commands.ManipulatorCommands.IntakeCoralBetter;
import frc.robot.Commands.ManipulatorCommands.RunAlgaeManipulator;
import frc.robot.Commands.ManipulatorCommands.RunManipulator;
import frc.robot.Commands.ManipulatorCommands.SpitPeice;
import frc.robot.Commands.ManipulatorCommands.SpitPeiceWithTime;
import frc.robot.Commands.MechanismCommands.MechanismToLoading;
import frc.robot.Commands.MechanismCommands.SetMechanismToPose;
import frc.robot.Commands.MechanismCommands.SetMechanismToPoseAuto;
import frc.robot.Commands.MechanismCommands.StowMechanismWithCoral;
import frc.robot.Commands.SwingArmCommands.SetSwingArm;
import frc.robot.Commands.SwingArmCommands.SetSwingArmAuto;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Lib.Utils.SwervePOILogic;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ManipulatorSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.SwingArmSubsystem;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class RobotContainer {
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  public static final GenericHID offsetOI = new GenericHID(2);

  private final SwerveSubsystem mSwerve = new SwerveSubsystem();
  private final ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  private final ManipulatorSubsystem mManipulatorSubsystem = new ManipulatorSubsystem();
  private final SwingArmSubsystem mSwingArmSubsystem = new SwingArmSubsystem();
  private final ClimberSubsystem mClimberSubsystem = new ClimberSubsystem();
  private final LEDSubsystem mLedSubsystem = new LEDSubsystem();

  public static boolean enableOffsets = false;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //Put all NamedCommands here
    NamedCommands.registerCommand("Elevator L4", new SetMechanismToPoseAuto(4.75, 0.31, mSwingArmSubsystem, mElevatorSubsystem));//L4
    NamedCommands.registerCommand("Elevator Barge", new SetMechanismToPoseAuto(4.75, 0.38, mSwingArmSubsystem, mElevatorSubsystem));
    NamedCommands.registerCommand("Stow Elevator", new StowMechanismWithCoral(mElevatorSubsystem, mSwingArmSubsystem, mManipulatorSubsystem));
    NamedCommands.registerCommand("Spit Piece", new SpitPeice(-1, mManipulatorSubsystem));//runs the manipulator back wards for x amount of seconds
    NamedCommands.registerCommand("Stop Spit Piece", new RunManipulator(mManipulatorSubsystem, 0));
    NamedCommands.registerCommand("Run Manipulator", new RunManipulator(mManipulatorSubsystem, -1));
    NamedCommands.registerCommand("Loading Elevator", new SetSwingArmAuto(mSwingArmSubsystem, 0.165));
    NamedCommands.registerCommand("Intake Coral", new IntakeCoralBetter(mManipulatorSubsystem));
    NamedCommands.registerCommand("Elevator L2", new SetMechanismToPose(1.55, 0.34, mSwingArmSubsystem, mElevatorSubsystem));
    NamedCommands.registerCommand("Elevator Descore Low", new SetSwingArm(mSwingArmSubsystem, 0.35));
    NamedCommands.registerCommand("Intake Algae", new IntakeAlgae(mManipulatorSubsystem, 0.5));
    NamedCommands.registerCommand("Spit Algae", new RunAlgaeManipulator(mManipulatorSubsystem, -1));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureBindings();
  }

  private void configureBindings() {
    mSwerve.setDefaultCommand(new TeleopSwerve(mSwerve, ()-> driver.getRawAxis(0), ()-> -driver.getRawAxis(1), ()-> driver.getRawAxis(4), ()-> false));

    SmartDashboard.putNumber("offsetOI axis 0", offsetOI.getRawAxis(0));
    SmartDashboard.putNumber("offsetOI axis 1", offsetOI.getRawAxis(1));
    //driver.b().toggleOnTrue(new GoToSwervePose(mSwerve, new Pose2d(new Translation2d(11.4, 7.7), Rotation2d.fromDegrees(90))));

    driver.start().onTrue(new ResetGyro(mSwerve));

    driver.x().whileTrue(mSwerve.PathfindToPose(() -> SwervePOILogic.findNearestLeftScore()));
    driver.y().whileTrue(mSwerve.PathfindToPose(()-> SwervePOILogic.findNearestDescore()));
    driver.a().whileTrue(mSwerve.PathfindToPose(()-> SwervePOILogic.findNearestReverseDescore()));
    //driver.a().whileTrue(mSwerve.PathfindToPose(()-> findNearestLoad()));
    driver.b().whileTrue(mSwerve.PathfindToPose(()-> SwervePOILogic.findNearestRightScore()));
    driver.leftTrigger().whileTrue(mSwerve.PathfindToPose(()-> SwervePOILogic.findNearestFarLoad()));

    driver.leftBumper().whileTrue(mSwerve.PathfindToPose(()-> SwervePOILogic.findNearestCloseLoad()));
    driver.rightBumper().whileTrue(new RunManipulator(mManipulatorSubsystem, -1));

    driver.leftStick().toggleOnTrue(new IntakeAlgae(mManipulatorSubsystem, 0.5));
    driver.rightTrigger().whileTrue(new RunAlgaeManipulator(mManipulatorSubsystem, -1));

    //driver.leftStick().onTrue(new SetSwingArm(mSwingArmSubsystem, 0.165));//loading station
    //driver.leftStick().onTrue(new SetSwingArm(mSwingArmSubsystem, -0.13));
    operator.rightStick().onTrue(new MechanismToLoading(mManipulatorSubsystem, mSwingArmSubsystem));
    //driver.x().onTrue(new SetSwingArm(mSwingArmSubsystem, 0));

    //driver.y().onTrue(new ElevatorControl(mElevatorSubsystem, 1.70));//L2
    // driver.y().onTrue(new ElevatorControl(mElevatorSubsystem, 5.1));//max height
    // driver.start().onTrue(new ElevatorControl(mElevatorSubsystem, 3));
    driver.rightStick().onTrue(new StowMechanismWithCoral(mElevatorSubsystem, mSwingArmSubsystem, mManipulatorSubsystem));

    operator.x().onTrue(new SetMechanismToPose(0.1, 0.231, mSwingArmSubsystem, mElevatorSubsystem));//L2
    operator.y().onTrue(new SetMechanismToPose(1.752, 0.256, mSwingArmSubsystem, mElevatorSubsystem));//L3
    operator.rightBumper().onTrue(new SetMechanismToPose(4.66, 0.324, mSwingArmSubsystem, mElevatorSubsystem));//L4
    operator.leftBumper().onTrue(new SetMechanismToPose(1.55, -0.251, mSwingArmSubsystem, mElevatorSubsystem));
    operator.leftStick().onTrue(new SetSwingArm(mSwingArmSubsystem, 0.216));
    
    operator.povUp().onTrue(new SetMechanismToPose(4.77, 0.405, mSwingArmSubsystem, mElevatorSubsystem));//Barge
    operator.povDown().onTrue(new SetMechanismToPose(0.1, -0.3, mSwingArmSubsystem, mElevatorSubsystem));//L1
    operator.povLeft().onTrue(new SetMechanismToPose(1.55, 0.34, mSwingArmSubsystem, mElevatorSubsystem));//Descore high Algae

    driver.povDown().onTrue(new SetSwingArm(mSwingArmSubsystem, 0.2857));
    driver.povUp().onTrue(new SetSwingArm(mSwingArmSubsystem, 0.35));
    operator.rightTrigger().whileTrue(new ControlClimberManual(mClimberSubsystem, 0.8));
    operator.leftTrigger().whileTrue(new ControlClimberManual(mClimberSubsystem, -0.8));

    //driver.back().toggleOnTrue(new SetElevatorSpeed(mElevatorSubsystem));//this is for finding the kg for the elevator
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}