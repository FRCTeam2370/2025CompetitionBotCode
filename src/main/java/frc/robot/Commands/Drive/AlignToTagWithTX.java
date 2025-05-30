// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.proto.Photon;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.PhotonVision;
import frc.robot.Subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTagWithTX extends Command {
  private SwerveSubsystem mSwerve;
   private PIDController translationPID = new PIDController(0.1, 0, 0.00);
   private static DoubleSupplier joystickVal;
   private SlewRateLimiter joystickLimiter = new SlewRateLimiter(5);
   private boolean left = false;
  /** Creates a new AlignToTagWithTX. */
  public AlignToTagWithTX(SwerveSubsystem mSwerve, boolean left) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mSwerve = mSwerve;
    this.joystickVal = joystickVal;
    this.left = left;
    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double joystickValue = Math.abs(joystickVal.getAsDouble()) < 0.01 ? 0 : joystickVal.getAsDouble();
    //mSwerve.drive(new Translation2d(translationPID.calculate(-Math.max(-2, Math.min(2, LimelightHelpers.getTX("limelight") + 3))),0 /*joystickLimiter.calculate(joystickValue) * Constants.SwerveConstants.maxSpeed //translationPID.calculate(Math.max(-10, Math.min(10, SwerveSubsystem.distanceToTag() - 22))))*/),  0, false, true);
    if(PhotonVision.camera.getLatestResult().hasTargets()){
      if(left){
        mSwerve.drive(new Translation2d(translationPID.calculate(PhotonVision.GetBestTargetYawClamped(-2)),0),  0, false, true);
      }else{
        mSwerve.drive(new Translation2d(translationPID.calculate(PhotonVision.GetBestTargetYawClamped(21.5)),0),  0, false, true);
      }
    }else{
      mSwerve.drive(new Translation2d(0,0), 0, false, true);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //mSwerve.drive(new Translation2d(0,0), 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if(LimelightHelpers.getTX("limelight") >  -3.5 && LimelightHelpers.getTX("limelight") < -2.5){
    // if(left){
    //   if(PhotonVision.GetBestTargetYaw() + 5 < 0.5 && PhotonVision.GetBestTargetYaw() + 5 > -0.5){
    //     return true;
    //   }else{
    //     return false;
    //   }
    // }else{
    //   if(PhotonVision.GetBestTargetYaw() - 4.5 < 0.5 && PhotonVision.GetBestTargetYaw() - 4.5 > -0.5){
    //     return true;
    //   }else{
    //     return false;
    //   }
    // }
    return false;
  }
}
