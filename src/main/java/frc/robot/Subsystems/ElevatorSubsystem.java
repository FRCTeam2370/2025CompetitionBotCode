// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  public static TalonFX elevatorMotor = new TalonFX(Constants.ElevatorConstants.ElevatorID);
  
  public static TalonFXConfiguration elevatorConfiguration = new TalonFXConfiguration();

  private static final PositionDutyCycle elevatorPosCycle = new PositionDutyCycle(0);
  private static final PositionDutyCycle elevatorDownPosCycle = new PositionDutyCycle(0);
  private static final MotionMagicVoltage elevatorMagic = new MotionMagicVoltage(0);
  private static final VoltageOut elevatorVoltOut = new VoltageOut(0);

  public static double lastElevatorPosition; 

  //creates a public enum that can have 3 different states of the elevator to corespond to different control modes
  public static enum ElevatorState {
    Moving,
    Idle,
    HoldingPosition
  }

  public static ElevatorState mElevatorState = ElevatorState.Moving; 

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    configElevator();//resets the elevator and sends the configs to the motor
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position in OutputShaft Rotations", 
      KrakenToOutputShaft(elevatorMotor.getPosition().getValueAsDouble()));//sends the position of the elevator to the Dashboard
    
    SmartDashboard.putNumber("Elevator Voltage", elevatorMotor.getMotorVoltage().getValueAsDouble());

    switch(mElevatorState) {
      case Idle:
        if(KrakenToOutputShaft(elevatorMotor.getPosition().getValueAsDouble()) <= 0.13){
          elevatorMotor.set(0);
          break;
        }else{
          setElevatorPos(0.12);
        }
        break;
      case Moving:
        break;
      case HoldingPosition:
        setElevatorPos(KrakenToOutputShaft(elevatorMotor.getPosition().getValueAsDouble()));
    }
  }

  public static void setElevatorPos(double position){//PID control of the position of the elevator
    position = OutputShaftToKraken(position);
    elevatorMotor.setControl(elevatorPosCycle.withPosition(position));//in rotations of the motor
  }

  public static double getElevatorShaftRots(){
    return KrakenToOutputShaft(elevatorMotor.getPosition().getValueAsDouble());
  }

  public static void setElevatorPosWithMagic(double position){
    position = OutputShaftToKraken(position);
    elevatorMotor.setControl(elevatorMagic.withPosition(position));
  }

  public static void setElevatorVolt(double voltage){
    elevatorMotor.setControl(elevatorVoltOut.withOutput(voltage));
  }

  public static void setElevator(double speed){
    //Percent output of the elevator
    elevatorMotor.set(speed);
  }

  public static void stopElevator(){
    //stops the elevator
    elevatorMotor.set(0);
  }

  public static void configElevator(){
    //configures all of the values for PID and eventually Motion Magic --- Slot0 holds regular PID vals
    elevatorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    //SLOT0 CONFIGURATION --- NORMAL PID CONTROL
    elevatorConfiguration.Slot0.kP = 0.05;//needs more tunning with weight and maybe Motion Magic pls , 2.25
    elevatorConfiguration.Slot0.kI = 0.025;
    elevatorConfiguration.Slot0.kD = 0.0025;//0.62
    elevatorConfiguration.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfiguration.Slot0.kG = 0.032;//0.03

    elevatorConfiguration.Slot2.kP = 0.01;//needs more tunning with weight and maybe Motion Magic pls , 2.25
    elevatorConfiguration.Slot2.kI = 0;
    elevatorConfiguration.Slot2.kD = 0.001;//0.62
    elevatorConfiguration.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfiguration.Slot2.kG = 0.037;//0.03

    elevatorConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;

    //SLOT1 CONFIGURATION --- MOTION MAGIC CONTROL
    var slot1Config = elevatorConfiguration.Slot1;
    slot1Config.kS = 0.4;
    slot1Config.kV = 0.12;
    slot1Config.kA = 0.02;
    slot1Config.kP = 1;
    slot1Config.kI = 0;
    slot1Config.kD = 0;

    elevatorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    elevatorConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;

    // elevatorConfiguration.MotionMagic.MotionMagicAcceleration = 400;
    // elevatorConfiguration.MotionMagic.MotionMagicJerk = 4000;

    elevatorMotor.setPosition(0);
    elevatorConfiguration.CurrentLimits.StatorCurrentLimit = 40;

    elevatorPosCycle.Slot = 0;
    elevatorDownPosCycle.Slot = 2;
    elevatorMagic.Slot = 1;

    elevatorMotor.getConfigurator().apply(elevatorConfiguration);
  }

  private static double OutputShaftToKraken(double OutputRotations){
    double krakenVal = OutputRotations * 13.3333;//15.4
    return krakenVal;
  }

  private static double KrakenToOutputShaft(double KrakenRotations){
    double OutputVal = KrakenRotations / 13.3333;//15.4
    return OutputVal;
  }
}
