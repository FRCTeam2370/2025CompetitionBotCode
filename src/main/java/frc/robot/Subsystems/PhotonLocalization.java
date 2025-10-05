// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.io.File;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonLocalization extends SubsystemBase {
  public static PhotonCamera camera = new PhotonCamera(Constants.PhotonVisionConstants.cameraName);
  public static PhotonCamera camera2 = new PhotonCamera(Constants.PhotonVisionConstants.cameraName2);

  private Matrix<N3, N1> curSdDevs1;
  private Matrix<N3, N1> curSdDevs2;

  public static PhotonPoseEstimator poseEstimator;
  public static PhotonPoseEstimator poseEstimator2;

  public static Transform3d fieldToCamera;

  public static Field2d photonfield = new Field2d();
  
  private SwerveSubsystem mSwerveSubsystem;
  /** Creates a new PhotonLocalization. */
  public PhotonLocalization(SwerveSubsystem mSwerveSubsystem) {
    poseEstimator = new PhotonPoseEstimator(Constants.PhotonVisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.PhotonVisionConstants.camToRobot);
    poseEstimator2 = new PhotonPoseEstimator(Constants.PhotonVisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.PhotonVisionConstants.cam2ToRobot);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    this.mSwerveSubsystem = mSwerveSubsystem;
  }

  @Override
  public void periodic() {  
    // This method will be called once per scheduler run
    // for(PhotonPipelineResult change : camera.getAllUnreadResults()){
    //   poseEstimator.update(change);
    //   //photonfield.setRobotPose(poseEstimator.getReferencePose().getX(), poseEstimator.getReferencePose().getY(), poseEstimator.getReferencePose().getRotation().toRotation2d());
    // }
    
    // SmartDashboard.putData(photonfield);

    // var results = camera.getAllUnreadResults();
    // for (var result : results) {
    //   var multiTagResult = result.getMultiTagResult();
    //   if (multiTagResult.isPresent()) {
    //     fieldToCamera = multiTagResult.get().estimatedPose.best;
    //     SmartDashboard.putNumber("Field to camera x", fieldToCamera.getX());
    //     SmartDashboard.putNumber("Field to camera y", fieldToCamera.getY());
    //     SmartDashboard.putNumber("Field to camera Rotation", fieldToCamera.getRotation().getAngle());
    //   }
    // }
    //checkCamera(camera, poseEstimator, curSdDevs1);
    //checkCamera(camera2, poseEstimator2, curSdDevs2);
    
      
  }

  public static Translation2d getTranslation(){
    if(fieldToCamera != null){
      return new Translation2d(fieldToCamera.getMeasureX(), fieldToCamera.getMeasureY());
    }else{
      return new Translation2d();
    }
  }

  public void checkCamera(PhotonCamera camera, PhotonPoseEstimator estimator, Matrix<N3, N1> sdDevs){
    var results = camera.getAllUnreadResults();
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for(var change : results){
      if(change.hasTargets()){
        visionEst = estimator.update(change);
        
        mSwerveSubsystem.updateEstimatorWithPose(visionEst.get().estimatedPose, change.getTimestampSeconds(), getEstSdDevs(visionEst, change.getTargets(), estimator, sdDevs));

        // SIMPLY SINGLE TAG LOCALIZATION
          // var target = change.getBestTarget();
          // if (Constants.PhotonVisionConstants.kTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
          //   Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), Constants.PhotonVisionConstants.kTagLayout.getTagPose(target.getFiducialId()).get(), Constants.PhotonVisionConstants.camToRobot);
          //   SmartDashboard.putNumber("Photon Pose X", robotPose.getX());
          //   SmartDashboard.putNumber("Photon Pose Y", robotPose.getY());
          //   mSwerveSubsystem.updateEstimatorWithPose(robotPose, change.getTimestampSeconds());
          // }
      }
    }
  }

  private Matrix<N3, N1> getEstSdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> results, PhotonPoseEstimator estimator, Matrix<N3, N1> sdDevs){
    if(estimatedPose.isEmpty()){
      sdDevs = Constants.PhotonVisionConstants.kSingleTagStdDevs;
      return sdDevs;
    }else{
      var estSdDevs = Constants.PhotonVisionConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      for(var change : results){
        var tagPose = estimator.getFieldTags().getTagPose(change.getFiducialId());
        if(tagPose.isEmpty()) continue; 

        numTags ++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if(numTags == 0){
        sdDevs = Constants.PhotonVisionConstants.kSingleTagStdDevs;
      }else{
        avgDist /= numTags;
        
        if(numTags > 1){
          sdDevs = Constants.PhotonVisionConstants.kMultiTagStdDevs;
        }
        if(numTags == 1 && avgDist > 4){
          estSdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }else{
          estSdDevs = estSdDevs.times(1 + (avgDist * avgDist / 30));
          sdDevs = estSdDevs;
        }
      }
      return sdDevs;
    }
  }
}
