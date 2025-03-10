// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib.Utils;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Subsystems.SwerveSubsystem;

/** Add your docs here. */
public class SwervePOILogic {
    public static double findDistanceBetweenAB(Translation2d A, Translation2d B){
        double distance;
        distance = Math.sqrt(Math.pow(A.getX() - B.getX(), 2) + Math.pow(A.getY() - B.getY(), 2));
        return distance;
    }

    public static Pose2d findNearestLeftScore(){
        Pose2d currentPose = SwerveSubsystem.poseEstimator.getEstimatedPosition();
        Pose2d returnPose;

        double distance1 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.CLOSE_SCORE_LEFT.getTranslation());
        double distance2 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.CLOSE_LEFT_SCORE_LEFT.getTranslation());
        double distance3 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.CLOSE_RIGHT_SCORE_LEFT.getTranslation());
        double distance4 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.FAR_LEFT_SCORE_LEFT.getTranslation());
        double distance5 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.FAR_RIGHT_SCORE_LEFT.getTranslation());
        double distance6 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.FAR_SCORE_LEFT.getTranslation());

        List<Double> distances = Arrays.asList(distance1, distance2, distance3, distance4, distance5, distance6);

        double closest = Collections.min(distances);

        returnPose = 
            closest == distance1 ? Constants.BlueSidePoses.CLOSE_SCORE_LEFT : 
            closest == distance2 ? Constants.BlueSidePoses.CLOSE_LEFT_SCORE_LEFT :
            closest == distance3 ? Constants.BlueSidePoses.CLOSE_RIGHT_SCORE_LEFT : 
            closest == distance4 ? Constants.BlueSidePoses.FAR_LEFT_SCORE_LEFT :
            closest == distance5 ? Constants.BlueSidePoses.FAR_RIGHT_SCORE_LEFT :
            closest == distance6 ? Constants.BlueSidePoses.FAR_SCORE_LEFT : null;

        return returnPose;
    }

    public static Pose2d findNearestRightScore(){
        Pose2d currentPose = SwerveSubsystem.poseEstimator.getEstimatedPosition();
        Pose2d returnPose;

        double distance1 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.CLOSE_SCORE_RIGHT.getTranslation());
        double distance2 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.CLOSE_LEFT_SCORE_RIGHT.getTranslation());
        double distance3 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.CLOSE_RIGHT_SCORE_RIGHT.getTranslation());
        double distance4 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.FAR_LEFT_SCORE_RIGHT.getTranslation());
        double distance5 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.FAR_RIGHT_SCORE_RIGHT.getTranslation());
        double distance6 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.FAR_SCORE_RIGHT.getTranslation());

        List<Double> distances = Arrays.asList(distance1, distance2, distance3, distance4, distance5, distance6);

        double closest = Collections.min(distances);

        returnPose = 
            closest == distance1 ? Constants.BlueSidePoses.CLOSE_SCORE_RIGHT : 
            closest == distance2 ? Constants.BlueSidePoses.CLOSE_LEFT_SCORE_RIGHT :
            closest == distance3 ? Constants.BlueSidePoses.CLOSE_RIGHT_SCORE_RIGHT : 
            closest == distance4 ? Constants.BlueSidePoses.FAR_LEFT_SCORE_RIGHT :
            closest == distance5 ? Constants.BlueSidePoses.FAR_RIGHT_SCORE_RIGHT :
            closest == distance6 ? Constants.BlueSidePoses.FAR_SCORE_RIGHT : null;

        return returnPose;
    }

    public static Pose2d findNearestReverseDescore(){
        Pose2d currentPose = SwerveSubsystem.poseEstimator.getEstimatedPosition();
        Pose2d returnPose;

        double distance1 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.CLOSE_REVERSE_DESCORE.getTranslation());
        double distance2 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.CLOSE_LEFT_REVERSE_DESCORE.getTranslation());
        double distance3 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.CLOSE_RIGHT_REVERSE_DESCORE.getTranslation());
        double distance4 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.FAR_LEFT_REVERSE_DESCORE.getTranslation());
        double distance5 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.FAR_RIGHT_REVERSE_DESCORE.getTranslation());
        double distance6 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.FAR_REVERSE_DESCORE.getTranslation());

        List<Double> distances = Arrays.asList(distance1, distance2, distance3, distance4, distance5, distance6);

        double closest = Collections.min(distances);

        returnPose = 
            closest == distance1 ? Constants.BlueSidePoses.CLOSE_REVERSE_DESCORE : 
            closest == distance2 ? Constants.BlueSidePoses.CLOSE_LEFT_REVERSE_DESCORE :
            closest == distance3 ? Constants.BlueSidePoses.CLOSE_RIGHT_REVERSE_DESCORE : 
            closest == distance4 ? Constants.BlueSidePoses.FAR_LEFT_REVERSE_DESCORE :
            closest == distance5 ? Constants.BlueSidePoses.FAR_RIGHT_REVERSE_DESCORE :
            closest == distance6 ? Constants.BlueSidePoses.FAR_REVERSE_DESCORE : null;

        return returnPose;
    }

    public static Pose2d findNearestDescore(){
        Pose2d currentPose = SwerveSubsystem.poseEstimator.getEstimatedPosition();
        Pose2d returnPose;

        double distance1 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.CLOSE_DESCORE.getTranslation());
        double distance2 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.CLOSE_LEFT_DESCORE.getTranslation());
        double distance3 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.CLOSE_RIGHT_DESCORE.getTranslation());
        double distance4 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.FAR_LEFT_DESCORE.getTranslation());
        double distance5 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.FAR_RIGHT_DESCORE.getTranslation());
        double distance6 = findDistanceBetweenAB(currentPose.getTranslation(), Constants.BlueSidePoses.FAR_DESCORE.getTranslation());

        List<Double> distances = Arrays.asList(distance1, distance2, distance3, distance4, distance5, distance6);

        double closest = Collections.min(distances);

        returnPose = 
            closest == distance1 ? Constants.BlueSidePoses.CLOSE_DESCORE : 
            closest == distance2 ? Constants.BlueSidePoses.CLOSE_LEFT_DESCORE :
            closest == distance3 ? Constants.BlueSidePoses.CLOSE_RIGHT_DESCORE : 
            closest == distance4 ? Constants.BlueSidePoses.FAR_LEFT_DESCORE :
            closest == distance5 ? Constants.BlueSidePoses.FAR_RIGHT_DESCORE :
            closest == distance6 ? Constants.BlueSidePoses.FAR_DESCORE : null;

        return returnPose;
    }
}
