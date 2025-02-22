// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
  public static SparkMax manipulatorMotor;
  public static SparkMax algaeMotor;
  public static SparkMaxConfig algaeConfig = new SparkMaxConfig();
  /** Creates a new ManipulatorSubsystem. */
  public ManipulatorSubsystem() {
    manipulatorMotor = new SparkMax(Constants.ManipulatorConstants.ManipulatorID, MotorType.kBrushless);
    algaeMotor = new SparkMax(Constants.ManipulatorConstants.AlgaeMotorID, MotorType.kBrushless);

    configureAlgaeMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void runManipulator(double speed){
    manipulatorMotor.set(speed);
  }

  public static void runAlgaeIntake(double speed){
    algaeMotor.set(speed);
  }

  private static void configureAlgaeMotor(){
    algaeConfig.idleMode(IdleMode.kBrake);
    
    algaeMotor.configure(algaeConfig, null, PersistMode.kPersistParameters);
  }
}
