// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;
import frc.robot.Lib.Utils.SwervePOILogic;

public class SwerveSubsystem extends SubsystemBase {
  public static Pigeon2 gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
  private static Pigeon2Configuration gyroConfig = new Pigeon2Configuration();
  public SwerveModule[] mSwerveModules;
  public static SwerveDriveOdometry odometry;

  public static PIDController rotationPIDauto = new PIDController(0.055, 0.00, 0.00);
  public static PIDController rotationPID = new PIDController(0.5, 0, 0);

  public static Field2d field = new Field2d();

  public static SwerveDrivePoseEstimator poseEstimator;

  public static Optional<Alliance> color;

  private static ChassisSpeeds lastSpeeds;

  private static Pose2d limelightPose, limelight2Pose;

  StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

  StructArrayPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyPose", Pose2d.struct).publish();

  public static double descoreCompare; 
  public static double reverseDescoreCompare;
  public static boolean closerToDescore;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    color = DriverStation.getAlliance();
    SmartDashboard.putString("Alliace Color", color.toString());

    rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    configureGyro();

    mSwerveModules = new SwerveModule[] {
      new SwerveModule(0, Constants.FLConstants.FLConstants),
      new SwerveModule(1, Constants.FRConstants.FRConstants),
      new SwerveModule(2, Constants.BLConstants.BLConstants),
      new SwerveModule(3, Constants.BRConstants.BRConstants)
    };

    

    LimelightHelpers.setPipelineIndex("limelight", 0);
    LimelightHelpers.setPipelineIndex("limelight-two", 0);

    odometry = new SwerveDriveOdometry(Constants.SwerveConstants.kinematics, gyro.getRotation2d(), getModulePositions());

    configurePathPlanner();

    if(isBlue()){
      poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.kinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), gyro.getRotation2d()));
    }else{
      poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.kinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), gyro.getRotation2d()));
    }


    resetGyro();
    if(SwerveSubsystem.isBlue()){
      resetOdometry(new Pose2d(SwerveSubsystem.poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(SwerveSubsystem.gyro.getYaw().getValueAsDouble() + 90)));
    }else{
      resetOdometry(new Pose2d(SwerveSubsystem.poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(SwerveSubsystem.gyro.getYaw().getValueAsDouble() + 270)));
    }
    
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Mod 0 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[0].getCANcoder().getDegrees()).getDegrees());
    // SmartDashboard.putNumber("Mod 1 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[1].getCANcoder().getDegrees()).getDegrees());
    // SmartDashboard.putNumber("Mod 2 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[2].getCANcoder().getDegrees()).getDegrees());
    // SmartDashboard.putNumber("Mod 3 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[3].getCANcoder().getDegrees()).getDegrees());

    // SmartDashboard.putNumber("Mod 0 Rot Pose", mSwerveModules[0].getTurnMotorRotation());
    // SmartDashboard.putNumber("Mod 1 Rot Pose", mSwerveModules[1].getTurnMotorRotation());
    // SmartDashboard.putNumber("Mod 2 Rot Pose", mSwerveModules[2].getTurnMotorRotation());
    // SmartDashboard.putNumber("Mod 3 Rot Pose", mSwerveModules[3].getTurnMotorRotation());

    // SmartDashboard.putNumber("Mod 0 drive Pose", mSwerveModules[0].getDriveMotorRotation());
    // SmartDashboard.putNumber("Mod 1 drive Pose", mSwerveModules[1].getDriveMotorRotation());
    // SmartDashboard.putNumber("Mod 2 drive Pose", mSwerveModules[2].getDriveMotorRotation());
    // SmartDashboard.putNumber("Mod 3 drive Pose", mSwerveModules[3].getDriveMotorRotation());

    SmartDashboard.putNumber("Wheel MPS", mSwerveModules[0].getWheelMPS());
    SmartDashboard.putNumber("Wheel Meters", mSwerveModules[0].getModuleMeters());

    SmartDashboard.putNumber("wheel Velocity", mSwerveModules[0].getWheelVelocity());
    SmartDashboard.putNumber("drive Voltage", mSwerveModules[0].getWheelVoltage());

    SmartDashboard.putNumber("Gyro Val", gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("pose x", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("pose y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("pose rot", poseEstimator.getEstimatedPosition().getRotation().getDegrees());

    // SmartDashboard.putNumber("Odometry x", odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("Odometry y", odometry.getPoseMeters().getY());

    //SmartDashboard.putNumber("limelight tx", LimelightHelpers.getTX("limelight"));

    updateOdometry();
    //odometry.update(getRotation2d(), getModulePositions());//USE THIS WHEN TESTING AUTOS WITHOUT FIELD LOCALIZATION
    resetOdometry(poseEstimator.getEstimatedPosition());

    field.setRobotPose(poseEstimator.getEstimatedPosition());

    posePublisher.set(new Pose2d[] {poseEstimator.getEstimatedPosition()});

    descoreCompare = Math.abs(SwervePOILogic.findNearestDescore().getFirst().getRotation().getDegrees() - poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    reverseDescoreCompare = Math.abs(SwervePOILogic.findNearestReverseDescore().getFirst().getRotation().getDegrees() - poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    closerToDescore = descoreCompare < reverseDescoreCompare;

    SmartDashboard.putNumber("descoreCompare", descoreCompare);
    SmartDashboard.putNumber("reverseDescoreCompare", reverseDescoreCompare);
    SmartDashboard.putBoolean("closertodescore", descoreCompare < reverseDescoreCompare);
  }

  public static Pair<Double, Double> findBestDescore(){
    double elevatorPos;

    if(closerToDescore){
      SwingArmSubsystem.swingArmPos = Constants.SwingArmConstants.reverseDescoreVal;
    }else if(!closerToDescore){
      SwingArmSubsystem.swingArmPos = Constants.SwingArmConstants.forwardDescoreVal;
    }else{
      SwingArmSubsystem.swingArmPos = Constants.SwingArmConstants.reverseDescoreVal;
    }
    
    if(SwervePOILogic.findNearestDescore().getSecond()){
      elevatorPos = Constants.ElevatorConstants.highDescoreVal;
    }else{
      elevatorPos = Constants.ElevatorConstants.lowDescoreVal;
    }
    
    return Pair.of(SwingArmSubsystem.swingArmPos, elevatorPos);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.kinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotationPID.calculate(rotation), getRotation2d()) :
      new ChassisSpeeds(translation.getX(), translation.getY(), rotationPIDauto.calculate(rotation))
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    mSwerveModules[0].setDesiredState(swerveModuleStates[0], isOpenLoop);
    mSwerveModules[1].setDesiredState(swerveModuleStates[1], isOpenLoop);
    mSwerveModules[2].setDesiredState(swerveModuleStates[2], isOpenLoop);
    mSwerveModules[3].setDesiredState(swerveModuleStates[3], isOpenLoop);
    publisher.set(swerveModuleStates);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[]{
      mSwerveModules[0].getPosition(),
      mSwerveModules[1].getPosition(),
      mSwerveModules[2].getPosition(),
      mSwerveModules[3].getPosition()
    };
    return positions;
  }

  public static boolean isBlue(){
    color = DriverStation.getAlliance();
    if(color.isPresent()){
      return color.get() == Alliance.Red ? false : true;
    }else{
      return true;
    }
  }

  public static void configureGyro(){
    resetGyro();
  }
  public static void resetGyro(){
    gyro.setYaw(180);//270
        
  }

  public Rotation2d getYaw(){
    new Rotation2d();
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  public double getHeading(){
    if(isBlue()){
      return Math.IEEEremainder(gyro.getRotation2d().getDegrees(), 360);
    }else{
      return Math.IEEEremainder(gyro.getRotation2d().getDegrees() + 180, 360);
    }
    
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);//pose.getRotation()
    poseEstimator.resetPose(pose);
  }

  public void updateOdometry(){
    poseEstimator.update(getYaw(), getModulePositions());

    boolean doRejectUpdate = false;
    Pose2d botPose;
    double botTimestamp;

    boolean usell1 = false;
    boolean usell2 = false;

    Pose2d ll1pose = getLimelight1Pose().getFirst();
    Pose2d ll2pose = getLimelight2Pose().getFirst();

    double ll1timestamp = getLimelight1Pose().getSecond();
    double ll2timestamp = getLimelight2Pose().getSecond();

    if(ll1pose == null && ll2pose == null){
      doRejectUpdate = true;
    }
    if(ll1pose != null){
      usell1 = true;
    }
    if(ll2pose != null){
      usell2 = true;
    }
    if(Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > 720){
      doRejectUpdate = true;
    }
    
    if(!doRejectUpdate){
      if(usell1 && usell2){
        Pose2d avepose = new Pose2d(new Translation2d((ll1pose.getX() + ll2pose.getX()) / 2, (ll1pose.getY() + ll2pose.getY()) / 2), poseEstimator.getEstimatedPosition().getRotation());
        botPose = avepose;
        // SmartDashboard.putNumber("bot Pose limelight average x", botPose.getX());
        // SmartDashboard.putNumber("bot Pose limelight average y", botPose.getY());
        // SmartDashboard.putNumber("bot Pose limelight average rot", botPose.getRotation().getDegrees());
        // SmartDashboard.putNumber("limelight 1 x", ll1pose.getX());
        // SmartDashboard.putNumber("limelight 1 y", ll1pose.getY());
        // SmartDashboard.putNumber("limelight 1 rot", ll1pose.getRotation().getDegrees());
        // SmartDashboard.putNumber("limelight 2 x", ll2pose.getX());
        // SmartDashboard.putNumber("limelight 2 y", ll2pose.getY());
        // SmartDashboard.putNumber("limelight 2 rot", ll2pose.getRotation().getDegrees());
  
        botTimestamp = (ll1timestamp + ll2timestamp) / 2; 
        poseEstimator.addVisionMeasurement(botPose, botTimestamp);
      }else if(usell1 && usell2 == false){
        poseEstimator.addVisionMeasurement(ll1pose, ll1timestamp);
      }else if(usell2 && usell1 == false){
        poseEstimator.addVisionMeasurement(ll2pose, ll2timestamp);
      }
      
    }
  }

  public static Pair<Pose2d, Double> getLimelight1Pose(){
    boolean doRejectUpdate = false;
    Pose2d returnPose = null;
    double returnTime = 0;

    LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    if(mt2 != null){
      if(Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > 720){
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0){
        doRejectUpdate = true;
      }
      if(!doRejectUpdate){
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        returnPose = mt2.pose;
        returnTime = mt2.timestampSeconds;
      }
    }

    return Pair.of(returnPose, returnTime);
  }

  public Pair<Pose2d, Double> getLimelight2Pose(){
    boolean doRejectUpdate = false;
    Pose2d returnPose = null;
    double returnTime = 0;

    LimelightHelpers.SetRobotOrientation("limelight-two", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2_otherOne = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-two");

    if(mt2_otherOne != null){
      if(Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > 720){
        doRejectUpdate = true;
      }
      if(mt2_otherOne.tagCount == 0){
        doRejectUpdate = true;
      }
      if(!doRejectUpdate){
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        returnPose = mt2_otherOne.pose;
        returnTime = mt2_otherOne.timestampSeconds;
      }
    } 

    return Pair.of(returnPose, returnTime);
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();//poseEstimator.getEstimatedPosition();
  }

  public Pose2d getEstimatedPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return Constants.SwerveConstants.kinematics.toChassisSpeeds(
      mSwerveModules[0].getState(),
      mSwerveModules[1].getState(),
      mSwerveModules[2].getState(),
      mSwerveModules[3].getState()
    );
  }

  public void goToPose(Pose2d pose){

  }

  public Command PathfindToPose(Supplier<Pose2d> poseSupplier){
    return new DeferredCommand(()-> AutoBuilder.pathfindToPose(poseSupplier.get(), Constants.SwerveConstants.telePathConstraints), Set.of(this));
  }

  public Command PathfindThenFollow(PathPlannerPath path){
    path.preventFlipping = false;
    return new DeferredCommand(()-> AutoBuilder.pathfindThenFollowPath(path, Constants.SwerveConstants.telePathConstraints), Set.of(this));
  }

  public Command followPath(PathPlannerPath path){
    return AutoBuilder.followPath(path);
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    // speeds.omegaRadiansPerSecond /= Constants.SwerveConstants.maxAngularVelocity;
    // speeds.omegaRadiansPerSecond = -speeds.omegaRadiansPerSecond;

    // ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    // SmartDashboard.putNumber("translation speed x", speeds.vxMetersPerSecond);
    // SmartDashboard.putNumber("translation speed y", speeds.vyMetersPerSecond);

    // SwerveModuleState[] states = Constants.SwerveConstants.kinematics.toSwerveModuleStates(targetSpeeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.maxSpeed);

    // mSwerveModules[0].setDesiredState(states[0], false);
    // mSwerveModules[1].setDesiredState(states[1], false);
    // mSwerveModules[2].setDesiredState(states[2], false);
    // mSwerveModules[3].setDesiredState(states[3], false);
    
    // if(Math.abs(speeds.vxMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.005) && Math.abs(speeds.vyMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.005)){
    //   drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), -speeds.omegaRadiansPerSecond / Constants.SwerveConstants.maxAngularVelocity, false, true);
    // }else{
    //   drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), -speeds.omegaRadiansPerSecond / Constants.SwerveConstants.maxAngularVelocity, false, false);
    // }

    drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), -speeds.omegaRadiansPerSecond / Constants.SwerveConstants.maxAngularVelocity, false, false);
  }

  public void configurePathPlanner(){
      RobotConfig config = new RobotConfig(74.088, 6.883, new ModuleConfig(0.048, 5.21208, 1.200, DCMotor.getKrakenX60(1), 60, 1), Constants.SwerveConstants.kinematics.getModules());
      try{
        config = RobotConfig.fromGUISettings();
        System.out.println("---------------------------Configured from GUI---------------------------");
      }catch(Exception exception){
        exception.printStackTrace();
        System.out.println("Doesn't like to config from gui settings");
      }
      AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              (speeds, feedforwards) -> driveRobotRelative(speeds),//drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), -speeds.omegaRadiansPerSecond / Constants.SwerveConstants.maxAngularVelocity, false, false),//drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond / 3.1154127, false, true),//driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(5, 0.0, 0.025), // Translation PID constants// 3.8 - p
                      new PIDConstants(5, 0.0, 0.0) // Rotation PID constants//kp 0.00755, ki 0.0001
              ),
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                if (color.isPresent()) {
                  return color.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[]{
      mSwerveModules[0].getState(),
      mSwerveModules[1].getState(),
      mSwerveModules[2].getState(),
      mSwerveModules[3].getState()
    };
    return states;
  }
}
