// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  public final SparkMax Wrist;
  public final SparkMax Arm;

  private final SparkMaxConfig WristConfig;
  private final SparkMaxConfig ArmConfig;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    WristConfig = new SparkMaxConfig();
    ArmConfig = new SparkMaxConfig();

    Wrist = new SparkMax(5, MotorType.kBrushless);
    Arm = new SparkMax(6, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoders(){
    Wrist.getEncoder().setPosition(0);
    Arm.getEncoder().setPosition(0);
  }

  public void moveWristForward(){
    Wrist.setVoltage(5);
  }
  public void moveWristBackward(){
    Wrist.setVoltage(-5);
  }
  
  public void moveArmForward(){
    Arm.setVoltage(5);
  }
  public void moveArmBackward(){
    Arm.setVoltage(-5);
  }
}
  