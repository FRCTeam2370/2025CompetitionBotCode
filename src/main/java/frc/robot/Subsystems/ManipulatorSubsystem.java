// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.MsvcRuntimeException;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
  public static SparkMax manipulatorMotor;
  public static SparkMax algaeMotor;
  public static CANrange manipulatorSensor;

  public static CANrangeConfiguration mSensorConfig = new CANrangeConfiguration();

  public static SparkMaxConfig algaeConfig = new SparkMaxConfig();
  public static SparkMaxConfig coralConfig = new SparkMaxConfig();

  public static boolean hasAlgae = false;
  /** Creates a new ManipulatorSubsystem. */
  public ManipulatorSubsystem() {
    manipulatorMotor = new SparkMax(Constants.ManipulatorConstants.ManipulatorID, MotorType.kBrushless);
    algaeMotor = new SparkMax(Constants.ManipulatorConstants.AlgaeMotorID, MotorType.kBrushless);

    manipulatorSensor = new CANrange(Constants.ManipulatorConstants.ManipulatorSensorID);

    configureAlgaeMotor();
    configCoral();
    configureManipulator();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Manipulator Sensor Val", manipulatorSensor.getDistance().getValueAsDouble());

    SmartDashboard.putBoolean("Has Coral", hasCoral());

    SmartDashboard.putNumber("Algae Motor Resistence", algaeMotor.getOutputCurrent());

    SmartDashboard.putBoolean("has Algae", hasAlgae);

    if(hasAlgae){
      runAlgaeIntake(0.15);
    }
  }

  public static boolean hasCoral(){
    return manipulatorSensor.getDistance().getValueAsDouble() < 0.040 ? true : false;
  }

  public static void runManipulator(double speed){
    manipulatorMotor.set(speed);
  }

  public static void runAlgaeIntake(double speed){
    algaeMotor.set(speed);
  }

  public static void runManipulatorFor(double repeats, double speed){
    for(int i = 0; i <= repeats; i++){
      runManipulator(speed);
    }
  }

  private static void configureAlgaeMotor(){
    algaeConfig.idleMode(IdleMode.kBrake);
    algaeConfig.inverted(false);
    
    algaeMotor.configure(algaeConfig, null, PersistMode.kPersistParameters);
  }

  private static void configCoral(){
    coralConfig.inverted(true);

    manipulatorMotor.configure(coralConfig, null, PersistMode.kPersistParameters);
  }

  private static void configureManipulator(){
    mSensorConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;

    manipulatorSensor.getConfigurator().apply(mSensorConfig);
  }
}
