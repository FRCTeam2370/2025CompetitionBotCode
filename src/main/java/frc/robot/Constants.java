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
import edu.wpi.first.wpilibj.Encoder;

/** Add your docs here. */
public class Constants {
    public static class ElevatorConstants {
        public static final int ElevatorID = 14;
        public static final double maxElevatorHeight = 4.75;
    }

    public static class SwingArmConstants {
        public static final int SwingArmID = 15;
        public static final int SwingArmEncoderID = 16;

        public static final double SwingArmOffset = 0.5202636;

        public static final double SwingArmMax = 0.4672;
        public static final double SwingArmMin = -0.3447;
    }

    public static class ManipulatorConstants {
        public static final int ManipulatorID = 18;
        public static final int AlgaeMotorID = 17;

        public static final int ManipulatorSensorID = 19;
    }

    public static class ClimberConstants {
        public static final int climberMotorID = 20;
    }

    public static class SwerveConstants {
        public static final PathConstraints telePathConstraints = new PathConstraints(2, 2, Math.toRadians(540.000), Math.toRadians(720.000));
        public static final double DrivekP = 0.02;
        public static final double DrivekI = 0;
        public static final double DrivekD = 0.001;
        public static final double DriveKS = 0.15;//for finding the kv and ks based off of each other -> 10 vel at 1.45 volt -> 1.45 - 0.3 = 1.15 / 10 = 0.115
        public static final double DriveKV = 0.0;//0.125

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
        public static final Pose2d REDLEFTLOADING = new Pose2d(16.05,0.55, Rotation2d.fromDegrees(220.5));// May need some tuning
        public static final Pose2d REDRIGHTLOADING = new Pose2d();

        public static final Pose2d REDFRONTSCORE = new Pose2d();
        public static final Pose2d REDFRONTLEFTSCORERIGHT = new Pose2d(13.841,3.10, Rotation2d.fromDegrees(34.5));
        public static final Pose2d REDBACKLEFTSCORE = new Pose2d(12.66, 2.98, Rotation2d.fromDegrees(60));

        public static final Pose2d REDPROCESSOR = new Pose2d(11.49, 7.408, Rotation2d.fromDegrees(180));
    }

    //MODULE 1
    public static class FRConstants {
        public static final int driveMotorID = 21;
        public static final int turnMotorID = 22;
        public static final int CANCoderID = 23;

        public static final InvertedValue driveInverted = InvertedValue.CounterClockwise_Positive;
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

        public static final InvertedValue driveInverted = InvertedValue.Clockwise_Positive;
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

        public static final InvertedValue driveInverted = InvertedValue.CounterClockwise_Positive;
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

        public static final InvertedValue driveInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue turnInverted = InvertedValue.Clockwise_Positive;

        public static final Rotation2d EncoderOffset = Rotation2d.fromDegrees(318.07617);
        public static final SensorDirectionValue EncoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final SwerveModuleConstants BLConstants = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, driveInverted, turnInverted, EncoderOffset, EncoderReversed);
    }
}
