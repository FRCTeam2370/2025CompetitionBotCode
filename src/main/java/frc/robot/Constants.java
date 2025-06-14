// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;

import frc.robot.Lib.Utils.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Encoder;

/** Add your docs here. */
public class Constants {
    public static class ElevatorConstants {
        public static final int ElevatorID = 14;
        public static final double maxElevatorHeight = 4.75;

        public static final double highDescoreVal = 1.55;
        public static final double lowDescoreVal = 0.15;
    }

    public static class SwingArmConstants {
        public static final int SwingArmID = 15;
        public static final int SwingArmEncoderID = 16;

        public static final double SwingArmOffset = 0.672;//old offset 0.5202636, 0.6665039

        public static final double SwingArmMax = 0.4672;
        public static final double SwingArmMin = -0.3447;

        
        public static final double forwardDescoreVal = 0.34;
        public static final double reverseDescoreVal = -0.34;
    }

    public static class ManipulatorConstants {
        public static final int ManipulatorID = 18;
        public static final int AlgaeMotorID = 17;

        public static final int ManipulatorSensorID = 19;
    }

    public static class ClimberConstants {
        public static final int climberMotorID = 20;
    }

    public static class PhotonVisionConstants {
        public static final double cameraHeight = Units.inchesToMeters(11.75);

        public static final double ReefTagHeights = Units.inchesToMeters(12);
    }

    public static class SwerveConstants {
        public static final PathConstraints telePathConstraints = new PathConstraints(2.5, 2, Math.toRadians(540.000), Math.toRadians(720.000));
        public static final double DrivekP = 0.02;
        public static final double DrivekI = 0;
        public static final double DrivekD = 0.00;
        public static final double DriveKS = 0.1;//for finding the kv and ks based off of each other -> 10 vel at 1.45 volt -> 1.45 - 0.3 = 1.15 / 10 = 0.115
        public static final double DriveKV = 0.0;//0.125//0.0105

        public static final double TurnkP = 4;
        public static final double TurnkI = 0;
        public static final double TurnkD = 0;

        public static final double driveRamp = 0.2;

        public static final double maxSpeed = 5.21208;//meters per second
        public static final double maxAngularVelocity = 2.25;//3.1154127;//radians per second

        public static final double wheelRadius = 3.75 / 2;
        public static final double wheelCircumference = (2 * Math.PI) * wheelRadius;
        public static final double wheelCircumferenceMeters = wheelCircumference * 0.0254;

        public static final int pigeonID = 1;

        public static final double trackWidth = 22.5;
        public static final double wheelBase = 20.5;

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d[]{
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
            }
        );
    }

    public static class RedSidePoses {
        public static final Pose2d RIGHT_LOADING_RIGHT = new Pose2d(16.16, 7.23, Rotation2d.fromDegrees(142 - 180));
        public static final Pose2d RIGHT_LOADING_LEFT = new Pose2d(17, 6.76, Rotation2d.fromDegrees(142 - 180));

        public static final Pose2d LEFT_LOADING_RIGHT = new Pose2d(16.75,1.15, Rotation2d.fromDegrees(39 - 180));// May need some tuning// 220.5
        public static final Pose2d LEFT_LOADING_LEFT = new Pose2d(16.14, 0.68, Rotation2d.fromDegrees(-143.12));

        public static final Pose2d CLOSE_SCORE_RIGHT = new Pose2d(14.3, 4.49, Rotation2d.fromDegrees(-90));
        public static final Pose2d CLOSE_SCORE_LEFT = new Pose2d(14.32, 4.08, Rotation2d.fromDegrees(-90));
        public static final Pose2d CLOSE_REVERSE_DESCORE = new Pose2d(14.32, 3.81, Rotation2d.fromDegrees(90));
        public static final Pose2d CLOSE_DESCORE = new Pose2d(14.325, 4.235, Rotation2d.fromDegrees(-90));

        public static final Pose2d CLOSE_LEFT_SCORE_RIGHT = new Pose2d(13.98, 3.13, Rotation2d.fromDegrees(-149));
        public static final Pose2d CLOSE_LEFT_SCORE_LEFT = new Pose2d(13.76, 3, Rotation2d.fromDegrees(-148));
        public static final Pose2d CLOSE_LEFT_REVERSE_DESCORE = new Pose2d(13.55, 2.68, Rotation2d.fromDegrees(33));
        public static final Pose2d CLOSE_LEFT_DESCORE = new Pose2d(13.87, 3.084, Rotation2d.fromDegrees(-150));

        public static final Pose2d FAR_LEFT_SCORE_RIGHT = new Pose2d(12.75, 2.75, Rotation2d.fromDegrees(148));
        public static final Pose2d FAR_LEFT_SCORE_LEFT = new Pose2d(12.55, 2.92, Rotation2d.fromDegrees(150));
        public static final Pose2d FAR_LEFT_REVERSE_DESCORE = new Pose2d(12.29, 3.07, Rotation2d.fromDegrees(-29.5));
        public static final Pose2d FAR_LEFT_DESCORE = new Pose2d(12.635, 2.844, Rotation2d.fromDegrees(150));

        public static final Pose2d FAR_SCORE_RIGHT = new Pose2d(11.84, 3.60, Rotation2d.fromDegrees(90));
        public static final Pose2d FAR_SCORE_LEFT = new Pose2d(11.82, 3.96, Rotation2d.fromDegrees(90));
        public static final Pose2d FAR_REVERSE_DESCORE = new Pose2d(11.90, 4.18, Rotation2d.fromDegrees(-90));
        public static final Pose2d FAR_DESCORE = new Pose2d(11.844, 3.767, Rotation2d.fromDegrees(90));
 
        public static final Pose2d FAR_RIGHT_SCORE_RIGHT = new Pose2d(12.11, 4.9, Rotation2d.fromDegrees(32.28));
        public static final Pose2d FAR_RIGHT_SCORE_LEFT = new Pose2d(12.4, 5.14, Rotation2d.fromDegrees(30));
        public static final Pose2d FAR_RIGHT_REVERSE_DESCORE = new Pose2d(12.76, 5.25, Rotation2d.fromDegrees(-149.7));
        public static final Pose2d FAR_RIGHT_DESCORE = new Pose2d(12.263, 4.99, Rotation2d.fromDegrees(30));
        
        public static final Pose2d CLOSE_RIGHT_SCORE_RIGHT = new Pose2d(13.33, 5.28, Rotation2d.fromDegrees(-31));
        public static final Pose2d CLOSE_RIGHT_SCORE_LEFT = new Pose2d(13.64, 5.07, Rotation2d.fromDegrees(-29));
        public static final Pose2d CLOSE_RIGHT_REVERSE_DESCORE = new Pose2d(13.92, 4.96, Rotation2d.fromDegrees(148));
        public static final Pose2d CLOSE_RIGHT_DESCORE = new Pose2d(13.522, 5.218, Rotation2d.fromDegrees(-30));

        public static final Pose2d PROCESSOR = new Pose2d(11.39, 7.3, Rotation2d.fromDegrees(-180));
    }

    public static class BlueSidePoses {
        public static final Pose2d RIGHT_LOADING_RIGHT = new Pose2d(1.47, 0.65, Rotation2d.fromDegrees(143));//-33.5
        public static final Pose2d RIGHT_LOADING_LEFT = new Pose2d(0.64, 1.23, Rotation2d.fromDegrees(143));

        public static final Pose2d LEFT_LOADING_RIGHT = new Pose2d(0.61,6.68, Rotation2d.fromDegrees(36));//-140.93
        public static final Pose2d LEFT_LOADING_LEFT = new Pose2d(1.42, 7.37, Rotation2d.fromDegrees(36));//-138// better

        public static final Pose2d CLOSE_SCORE_RIGHT = new Pose2d(3.23, 3.58, Rotation2d.fromDegrees(90));
        public static final Pose2d CLOSE_SCORE_LEFT = new Pose2d(3.24, 3.91, Rotation2d.fromDegrees(90));
        public static final Pose2d CLOSE_REVERSE_DESCORE = new Pose2d(3.3, 4.27, Rotation2d.fromDegrees(-90));
        public static final Pose2d CLOSE_DESCORE = new Pose2d(3.225, 3.82, Rotation2d.fromDegrees(90));

        public static final Pose2d CLOSE_LEFT_SCORE_RIGHT = new Pose2d(3.48, 4.85, Rotation2d.fromDegrees(33.02));
        public static final Pose2d CLOSE_LEFT_SCORE_LEFT = new Pose2d(3.81, 5.04, Rotation2d.fromDegrees(30));//35/02
        public static final Pose2d CLOSE_LEFT_REVERSE_DESCORE = new Pose2d(4.22, 5.2, Rotation2d.fromDegrees(-150));//-143.33
        public static final Pose2d CLOSE_LEFT_DESCORE = new Pose2d(3.668, 4.99, Rotation2d.fromDegrees(30));

        public static final Pose2d FAR_LEFT_SCORE_RIGHT = new Pose2d(4.75, 5.3, Rotation2d.fromDegrees(-29));
        public static final Pose2d FAR_LEFT_SCORE_LEFT = new Pose2d(5.11, 5.15, Rotation2d.fromDegrees(-30));
        public static final Pose2d FAR_LEFT_REVERSE_DESCORE = new Pose2d(5.22, 4.96, Rotation2d.fromDegrees(150));
        public static final Pose2d FAR_LEFT_DESCORE = new Pose2d(4.927, 5.218, Rotation2d.fromDegrees(-30));

        public static final Pose2d FAR_SCORE_RIGHT = new Pose2d(5.71, 4.42, Rotation2d.fromDegrees(-90));
        public static final Pose2d FAR_SCORE_LEFT = new Pose2d(5.72, 4.07, Rotation2d.fromDegrees(-90));
        public static final Pose2d FAR_REVERSE_DESCORE = new Pose2d(5.83, 3.78, Rotation2d.fromDegrees(90));
        public static final Pose2d FAR_DESCORE = new Pose2d(5.73, 4.235, Rotation2d.fromDegrees(-91.24));

        public static final Pose2d FAR_RIGHT_SCORE_RIGHT = new Pose2d(5.45, 3.21, Rotation2d.fromDegrees(-150));
        public static final Pose2d FAR_RIGHT_SCORE_LEFT = new Pose2d(5.13, 2.94, Rotation2d.fromDegrees(-149));
        public static final Pose2d FAR_RIGHT_REVERSE_DESCORE = new Pose2d(4.84, 2.89, Rotation2d.fromDegrees(36));
        public static final Pose2d FAR_RIGHT_DESCORE = new Pose2d(5.323, 3.06, Rotation2d.fromDegrees(-150));
        
        public static final Pose2d CLOSE_RIGHT_SCORE_RIGHT = new Pose2d(4.24, 2.74, Rotation2d.fromDegrees(145.83));
        public static final Pose2d CLOSE_RIGHT_SCORE_LEFT = new Pose2d(3.92, 2.86, Rotation2d.fromDegrees(149.83));
        public static final Pose2d CLOSE_RIGHT_REVERSE_DESCORE = new Pose2d(3.68, 3.12, Rotation2d.fromDegrees(-30));
        public static final Pose2d CLOSE_RIGHT_DESCORE = new Pose2d(4.052, 2.832, Rotation2d.fromDegrees(150));

        public static final Pose2d PROCESSOR = new Pose2d(6.12, 0.72, Rotation2d.fromDegrees(-180));
    }

    //MODULE 1
    public static class FRConstants {
        public static final int driveMotorID = 21;
        public static final int turnMotorID = 22;
        public static final int CANCoderID = 23;

        public static final InvertedValue driveInverted = InvertedValue.Clockwise_Positive;//counter clockwise
        public static final InvertedValue turnInverted = InvertedValue.Clockwise_Positive;

        public static final Rotation2d EncoderOffset = Rotation2d.fromDegrees(290.2148);
        public static final SensorDirectionValue EncoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final SwerveModuleConstants FRConstants = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, driveInverted, turnInverted, EncoderOffset, EncoderReversed);
    }

    //MODULE 0
    public static class FLConstants {
        public static final int driveMotorID = 11;
        public static final int turnMotorID = 12;
        public static final int CANCoderID = 13;

        public static final InvertedValue driveInverted = InvertedValue.CounterClockwise_Positive;//clockwise
        public static final InvertedValue turnInverted = InvertedValue.Clockwise_Positive;

        public static final Rotation2d EncoderOffset = Rotation2d.fromDegrees(173.5830);
        public static final SensorDirectionValue EncoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final SwerveModuleConstants FLConstants = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, driveInverted, turnInverted, EncoderOffset, EncoderReversed);
    }

    //MODULE 3
    public static class BRConstants {
        public static final int driveMotorID = 31;
        public static final int turnMotorID = 32;
        public static final int CANCoderID = 33;

        public static final InvertedValue driveInverted = InvertedValue.Clockwise_Positive;//counter clockwise
        public static final InvertedValue turnInverted = InvertedValue.Clockwise_Positive;

        public static final Rotation2d EncoderOffset = Rotation2d.fromDegrees(82.3535);
        public static final SensorDirectionValue EncoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final SwerveModuleConstants BRConstants = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, driveInverted, turnInverted, EncoderOffset, EncoderReversed);
    }

    //MODULE 2
    public static class BLConstants {
        public static final int driveMotorID = 41;
        public static final int turnMotorID = 42;
        public static final int CANCoderID = 43;

        public static final InvertedValue driveInverted = InvertedValue.CounterClockwise_Positive;//clockwise
        public static final InvertedValue turnInverted = InvertedValue.Clockwise_Positive;

        public static final Rotation2d EncoderOffset = Rotation2d.fromDegrees(318.07617);
        public static final SensorDirectionValue EncoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final SwerveModuleConstants BLConstants = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, driveInverted, turnInverted, EncoderOffset, EncoderReversed);
    }
}
