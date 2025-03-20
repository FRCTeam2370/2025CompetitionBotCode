// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LEDSubsystem extends SubsystemBase {
  public static DigitalOutput output1 = new DigitalOutput(6); //Output Channel is currently temporary filler numbers, will be switched in future
  public static DigitalOutput output2 = new DigitalOutput(7);
  public static DigitalOutput output3 = new DigitalOutput(8);
  public static DigitalOutput output4 = new DigitalOutput(9);
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("output 1", output1.get());
    SmartDashboard.putBoolean("output 2", output2.get());
    SmartDashboard.putBoolean("output 3", output3.get());
    SmartDashboard.putBoolean("output 4", output4.get());

    if(RobotContainer.testing.a().getAsBoolean()){
      hasCoralLED();
    }else if(RobotContainer.testing.b().getAsBoolean()){
      hasAlgaeLED();
    }else if(RobotContainer.testing.x().getAsBoolean()){
      obtainedPieceLED();
    }else if(RobotContainer.testing.y().getAsBoolean()){
      blueClimbingAnimLED();
    }

  }

  public static void obtainedPieceLED() {
    output1.set(false);
    output2.set(true);
    output3.set(true);
    output4.set(true);
  }

  public static void hasCoralLED() {
    output1.set(true);
    output2.set(false);
    output3.set(true);
    output4.set(true);
  }

  public static void hasAlgaeLED() {
    output1.set(false);
    output2.set(false);
    output3.set(true);
    output4.set(true);
  }

  public static void ableToScoreLED() {
    output1.set(true);
    output2.set(true);
    output3.set(false);
    output4.set(true);
  }

  public static void endgameWarningLED() {
    output1.set(false);
    output2.set(true);
    output3.set(false);
    output4.set(true);
  }

  public static void blueClimbingAnimLED() {
    output1.set(true);
    output2.set(false);
    output3.set(false);
    output4.set(true);
  }

  public static void redClimbingAnimLED() {
    output1.set(false);
    output2.set(false);
    output3.set(false);
    output4.set(true);
  }
}
