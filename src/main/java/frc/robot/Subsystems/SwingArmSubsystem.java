// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwingArmSubsystem extends SubsystemBase {
  public static TalonFX swingArmMotor;
  public static TalonFXConfiguration swingArmConfig = new TalonFXConfiguration();

  public static CANcoder swingArmEncoder;
  public static CANcoderConfiguration swingArmEncoderConfig = new CANcoderConfiguration();

  private static PositionDutyCycle swingArmPosCycle = new PositionDutyCycle(0);

  private static double offset = 0;
  /** Creates a new SwingArmSubsystem. */
  public SwingArmSubsystem() {
    swingArmMotor = new TalonFX(Constants.SwingArmConstants.SwingArmID);
    configSwingArm();

    swingArmEncoder = new CANcoder(Constants.SwingArmConstants.SwingArmEncoderID);
    configSwingArmEncoder();

    //resetArmToAbsolute();

    setSwingArmPos(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("swingArm Motor rotations", swingArmMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("swingArm Mechanism Rotations", getArmRotations());
    SmartDashboard.putNumber("SwingArm Absolut Encoder", swingArmEncoder.getAbsolutePosition().getValueAsDouble());

    offset = RobotContainer.offsetOI.getRawAxis(1) / 12;
    SmartDashboard.putNumber("swing arm offset", offset);
  }

  public static void setSwingArmPos(double pos){
    if(RobotContainer.enableOffsets){
      swingArmMotor.setControl(swingArmPosCycle.withPosition(armRotationsToKraken((pos + offset) > Constants.SwingArmConstants.SwingArmMax ? Constants.SwingArmConstants.SwingArmMax : (pos + offset) < Constants.SwingArmConstants.SwingArmMin ? Constants.SwingArmConstants.SwingArmMin : (pos + offset))));
    }else{
      swingArmMotor.setControl(swingArmPosCycle.withPosition(armRotationsToKraken(pos)));
    }
    
  }

  public static double getArmRotations(){
    return krakenToArmRotations(swingArmMotor.getPosition().getValueAsDouble());
  }

  private static void configSwingArm(){
    swingArmConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    swingArmConfig.Slot0.kP = 0.15;
    swingArmConfig.Slot0.kI = 0.015;
    swingArmConfig.Slot0.kD = 0.02;

    swingArmConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2;

    swingArmConfig.CurrentLimits.StatorCurrentLimit = 30;

    swingArmMotor.getConfigurator().apply(swingArmConfig);

    swingArmMotor.setNeutralMode(NeutralModeValue.Coast);

    swingArmMotor.setPosition(0);
  }

  private static void configSwingArmEncoder(){
    swingArmEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    swingArmEncoderConfig.MagnetSensor.MagnetOffset = 0;

    swingArmEncoder.getConfigurator().apply(swingArmEncoderConfig);
  }

  private static void resetArmToAbsolute(){
    double krakenPos = armRotationsToKraken(swingArmEncoder.getAbsolutePosition().waitForUpdate(0.5).getValueAsDouble() - Constants.SwingArmConstants.SwingArmOffset);
    swingArmMotor.setPosition(krakenPos);
  }

  //CONVERSIONS
  public static double krakenToArmRotations(double krakenRot){
    double output = krakenRot / 37.333333333333333;
    return output;
  }

  public static double armRotationsToKraken(double armRot){
    double output = armRot * 37.33333333333333;
    return output;
  }
}
