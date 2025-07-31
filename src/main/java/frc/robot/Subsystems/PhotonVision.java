// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class PhotonVision extends SubsystemBase {
  public static PhotonCamera camera = new PhotonCamera("PhotonCamera");
  /** Creates a new PhotonVision. */
  public PhotonVision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(camera.getLatestResult().hasTargets()){
    //   SmartDashboard.putNumber("PhotonCamera distance from target", PhotonUtils.calculateDistanceToTargetMeters(Constants.PhotonVisionConstants.cameraHeight, Constants.PhotonVisionConstants.ReefTagHeights, 0, camera.getLatestResult().getBestTarget().getPitch()));
    // }
    //SmartDashboard.putNumber("PhotonVisionYaw", GetBestTargetYawClamped(10));
  }

  public static double GetBestTargetYaw(){
    if(camera.getLatestResult().hasTargets()){
      double rVal = camera.getLatestResult().getBestTarget().getYaw();
      return rVal;
    }else{
      return 0;
    }
    
  }

  public static double GetBestTargetYawClamped(double offset){
    if(camera.getLatestResult().hasTargets() && camera.getLatestResult() != null){
      if(camera.getLatestResult().getBestTarget() != null){
        //double rVal = camera.getLatestResult().getBestTarget() != null ? camera.getLatestResult().getBestTarget().getYaw() : 0;// for future reference you probable don't need the .getBestTarget because there is only one target in frame
        double rVal = camera.getLatestResult().getBestTarget().getYaw();
        rVal = Math.max(-5, Math.min(5, rVal + offset));
        return rVal;
      }else{
        return 0;
      }
    }else{
      return 0;
    }
  }
}
