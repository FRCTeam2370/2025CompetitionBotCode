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
import frc.robot.Commands.Drive.AlignToTagWithTX;
import frc.robot.Commands.Drive.DriveRobotRelative;
import frc.robot.Commands.Drive.GoToSwervePose;
import frc.robot.Commands.Drive.TeleopSwerve;
import frc.robot.Commands.Drive.TestAllignToTarget;
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
import frc.robot.Commands.MechanismCommands.MechanismToDescore;
import frc.robot.Commands.MechanismCommands.MechanismToLoading;
import frc.robot.Commands.MechanismCommands.MechanismToLoadingAuto;
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
import frc.robot.Subsystems.PhotonLocalization;
import frc.robot.Subsystems.PhotonVision;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.SwingArmSubsystem;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class RobotContainer {
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  public static final GenericHID offsetOI = new GenericHID(2);
  public static final CommandXboxController testing = new CommandXboxController(3);

  private final SwerveSubsystem mSwerve = new SwerveSubsystem();
  private final ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  private final ManipulatorSubsystem mManipulatorSubsystem = new ManipulatorSubsystem();
  private final SwingArmSubsystem mSwingArmSubsystem = new SwingArmSubsystem();
  private final ClimberSubsystem mClimberSubsystem = new ClimberSubsystem();
  private final LEDSubsystem mLedSubsystem = new LEDSubsystem();
  private final PhotonVision mPhotonVision = new PhotonVision();
  private final PhotonLocalization mLocalization = new PhotonLocalization(mSwerve);

  public static boolean enableOffsets = false;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //Put all NamedCommands here
    NamedCommands.registerCommand("Elevator L4", new SetMechanismToPoseAuto(4.73, 0.315, mSwingArmSubsystem, mElevatorSubsystem));//L4
    NamedCommands.registerCommand("Elevator Barge", new SetMechanismToPoseAuto(4.72, 0.39, mSwingArmSubsystem, mElevatorSubsystem));
    NamedCommands.registerCommand("Elevator Barge Reverse", new SetMechanismToPoseAuto(4.72, -0.39, mSwingArmSubsystem, mElevatorSubsystem));
    NamedCommands.registerCommand("Elevator L2", new SetMechanismToPoseAuto(0.31, 0.248, mSwingArmSubsystem, mElevatorSubsystem));
    NamedCommands.registerCommand("Stow Elevator", new StowMechanismWithCoral(mElevatorSubsystem, mSwingArmSubsystem, mManipulatorSubsystem));
    
    NamedCommands.registerCommand("Loading Elevator", new SetMechanismToPoseAuto(0.1, -0.13, mSwingArmSubsystem, mElevatorSubsystem));//new SetSwingArmAuto(mSwingArmSubsystem, 0.165));
    NamedCommands.registerCommand("Reverse Loading Elevator", new MechanismToLoading(mManipulatorSubsystem, mSwingArmSubsystem, mLedSubsystem, mElevatorSubsystem));
    NamedCommands.registerCommand("Intake Coral", new IntakeCoralBetter(mManipulatorSubsystem, mLedSubsystem));
    
    NamedCommands.registerCommand("Elevator Descore Low", new SetSwingArm(mSwingArmSubsystem, 0.35));
    NamedCommands.registerCommand("Elevator Descore Reverse", new SetSwingArm(mSwingArmSubsystem, -0.35));
    NamedCommands.registerCommand("Descore High", new SetMechanismToPoseAuto(1.55, 0.34, mSwingArmSubsystem, mElevatorSubsystem));
    NamedCommands.registerCommand("Descore High Reverse", new SetMechanismToPoseAuto(1.55, -0.34, mSwingArmSubsystem, mElevatorSubsystem));
    
    NamedCommands.registerCommand("Spit Piece", new SpitPeice(-1, mManipulatorSubsystem));//runs the manipulator back wards for x amount of seconds
    NamedCommands.registerCommand("Stop Spit Piece", new RunManipulator(mManipulatorSubsystem, 0));
    NamedCommands.registerCommand("Run Manipulator", new RunManipulator(mManipulatorSubsystem, -1));
    NamedCommands.registerCommand("Intake Algae", new IntakeAlgae(mManipulatorSubsystem, 0.5));
    NamedCommands.registerCommand("Spit Algae", new RunAlgaeManipulator(mManipulatorSubsystem, -1));

    NamedCommands.registerCommand("AlignToLeft", new AlignToTagWithTX(mSwerve, true));
    NamedCommands.registerCommand("AlignToRight", new AlignToTagWithTX(mSwerve, false));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureBindings();
  }

  private void configureBindings() {
    mSwerve.setDefaultCommand(new TeleopSwerve(mSwerve, ()-> driver.getRawAxis(0), ()-> -driver.getRawAxis(1), ()-> driver.getRawAxis(4), ()-> false));

    SmartDashboard.putNumber("offsetOI axis 0", offsetOI.getRawAxis(0));
    SmartDashboard.putNumber("offsetOI axis 1", offsetOI.getRawAxis(1));


    // DRIVER CONTROLS


    // Start button: Gyro Reset
    driver.start().onTrue(new ResetGyro(mSwerve));
    
    // Right Bumper: Spit out coral
    driver.rightBumper().whileTrue(new RunManipulator(mManipulatorSubsystem, -1));

    // Left Mouse Button (extra button on the top of the controller in between the trigger and the bumper): Intake Algae
    driver.leftStick().toggleOnTrue(new IntakeAlgae(mManipulatorSubsystem, 0.5));
    // Right Trigger: Spit out Algae
    driver.rightTrigger().whileTrue(new RunAlgaeManipulator(mManipulatorSubsystem, -1));

    //D-PAD Right: Run Algae Manipulator 
    driver.povRight().whileTrue(new RunAlgaeManipulator(mManipulatorSubsystem, 0.5));

    // Right Mouse Button: Stow Elevator
    driver.rightStick().onTrue(new StowMechanismWithCoral(mElevatorSubsystem, mSwingArmSubsystem, mManipulatorSubsystem));

    // D-PAD Left: Set the elevator to the position to intake a piece from the loading station
    driver.povLeft().onTrue(new MechanismToLoadingAuto(mManipulatorSubsystem, mSwingArmSubsystem, mLedSubsystem, mElevatorSubsystem));

    // D-PAD Down: some random intermediate value between descore low and Process
    driver.povDown().onTrue(new StowMechanismWithCoral(mElevatorSubsystem, mSwingArmSubsystem, mManipulatorSubsystem).andThen(new SetSwingArm(mSwingArmSubsystem, 0.2857)));
    //D-PAD Up: Set the Elevator to Descore low algae
    driver.povUp().onTrue(new StowMechanismWithCoral(mElevatorSubsystem, mSwingArmSubsystem, mManipulatorSubsystem).andThen(new SetSwingArm(mSwingArmSubsystem, 0.35)));

    //AUTO ALIGN CONTROLS

    // X: Auto Align to closest left reef pole
    driver.x().whileTrue(mSwerve.PathfindToPose(() -> SwervePOILogic.findNearestLeftScore()));
    // Y: Auto Align to closest descore, robot facing forward
    driver.y().whileTrue(mSwerve.PathfindToPose(()-> SwervePOILogic.findNearestDescore().getFirst()));
    // A: Auto Align to closest descore, robot facing backwards
    driver.a().whileTrue(mSwerve.PathfindToPose(()-> SwervePOILogic.findNearestReverseDescore().getFirst()));
    // B: Auto Align to closest right reef pole
    driver.b().whileTrue(mSwerve.PathfindToPose(()-> SwervePOILogic.findNearestRightScore()));
    // Left Trigger: Auto Align to closer loading station on the outside of the field (aka closer to the other alliance)
    driver.leftTrigger().whileTrue(mSwerve.PathfindToPose(()-> SwervePOILogic.findNearestFarLoad()));
    // Left Bumper: Auto Align to closer loading station on the inside of the field (aka closer to the driverstations)
    driver.leftBumper().whileTrue(mSwerve.PathfindToPose(()-> SwervePOILogic.findNearestCloseLoad()));


    // OPERATOR CONTROLS


    // Right Mouse Button: Set the mechanism to the loading position 
    operator.rightStick().onTrue(new MechanismToLoadingAuto(mManipulatorSubsystem, mSwingArmSubsystem, mLedSubsystem, mElevatorSubsystem));
    
    // A: Sets mechanism for L1
    operator.a().onTrue(new SetMechanismToPose(0.1, -0.254, mSwingArmSubsystem, mElevatorSubsystem));
    // B: Sets the Elevator to the scoring hight L2
    operator.b().onTrue(new SetMechanismToPose(0.31, 0.25, mSwingArmSubsystem, mElevatorSubsystem));//swing arm 0.248
    // X: Sets the Elevator to the scoring hight L3
    operator.x().onTrue(new SetMechanismToPose(1.88, 0.267, mSwingArmSubsystem, mElevatorSubsystem));
    // Y: Sets the Elevator to the scoring hight L4
    operator.y().onTrue(new SetMechanismToPose(4.72, 0.3175, mSwingArmSubsystem, mElevatorSubsystem));//4.66, 0.324
    // Right Bumper: Sets mechanism for the Barge
    operator.rightBumper().onTrue(new SetMechanismToPose(4.73, 0.405, mSwingArmSubsystem, mElevatorSubsystem));
    
    // Left Bumper: Sets the mechanism to the correct Descore position based on robot field position
    operator.leftBumper().onTrue(new MechanismToDescore(mManipulatorSubsystem, mElevatorSubsystem, mSwingArmSubsystem, mLedSubsystem));
    
    // Left Mouse Button: Sets the Elevator for Processor
    operator.leftStick().onTrue(new StowElevator(mElevatorSubsystem).andThen(new SetSwingArm(mSwingArmSubsystem, 0.2)));

    // D-PAD Left: manual descore high
    operator.povLeft().onTrue(new SetMechanismToPose(1.55, 0.34, mSwingArmSubsystem, mElevatorSubsystem));
    // D-PAD Right: manual descore low
    operator.povRight().onTrue(new StowMechanismWithCoral(mElevatorSubsystem, mSwingArmSubsystem, mManipulatorSubsystem).andThen(new SetSwingArm(mSwingArmSubsystem, 0.35)));
    // D-PAD UP: Stow Mechanism
    operator.povUp().onTrue(new StowMechanismWithCoral(mElevatorSubsystem, mSwingArmSubsystem, mManipulatorSubsystem));

    // CLIMBING CONTROLS

    // Right Trigger: unspool climber
    operator.rightTrigger().whileTrue(new ControlClimberManual(mClimberSubsystem, 0.8));
    // Right Trigger: climb
    operator.leftTrigger().whileTrue(new ControlClimberManual(mClimberSubsystem, -0.8));

    //driver.back().toggleOnTrue(new SetElevatorSpeed(mElevatorSubsystem));//this is for finding the kg for the elevator
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}