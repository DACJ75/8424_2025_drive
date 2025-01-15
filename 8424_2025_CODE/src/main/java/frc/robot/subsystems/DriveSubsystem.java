// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  public final SparkMax frontRight;
  public final SparkMax backRight;
  public final SparkMax frontLeft;
  public final SparkMax backLeft;

  private final RelativeEncoder frontRightEncoder;
  private final RelativeEncoder frontLeftEncoder;

  private final SparkMaxConfig frontRightConfig;
  private final SparkMaxConfig frontLeftConfig;
  private final SparkMaxConfig backRightConfig;
  private final SparkMaxConfig backLeftConfig;

  private final DifferentialDrive diffDrive;

  public DriveSubsystem() {
    frontRight = new SparkMax(0, MotorType.kBrushless);
    backRight = new SparkMax(1, MotorType.kBrushless);
    frontLeft = new SparkMax(2, MotorType.kBrushless);
    backLeft = new SparkMax(3, MotorType.kBrushless);

    frontRightEncoder = frontRight.getEncoder();
    frontLeftEncoder = frontLeft.getEncoder();

    backRightConfig = new SparkMaxConfig();
    backLeftConfig = new SparkMaxConfig();
    frontRightConfig = new SparkMaxConfig();
    frontLeftConfig = new SparkMaxConfig();

    backRightConfig.follow(frontRight);
    backRightConfig.inverted(true);

    frontRightConfig.inverted(true);

    frontLeftConfig.inverted(false);

    backLeftConfig.follow(frontLeft);
    backLeftConfig.inverted(false);

    backLeft.configure(
        backLeftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    backRight.configure(backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRight.configure(frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontLeft.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    diffDrive = new DifferentialDrive(frontLeft, frontRight);
  }

  public void drive(double xspeed, double omega) {

    double maxSpeed = 0.6;
    double maxRotation = 0.6;
    double xModified = maxSpeed * Math.pow(xspeed, 3);

    diffDrive.arcadeDrive(xModified, maxRotation * omega);
  }

  public double getEncoderPosition(RelativeEncoder encoder) {
    return encoder.getPosition();
  }

  public void resetEncoders()
  {
    frontRight.getEncoder().setPosition(0);
    frontLeft.getEncoder().setPosition(0);
    backRight.getEncoder().setPosition(0);
    backLeft.getEncoder().setPosition(0);
  }

  public Pose2d getPose() {

    return new Pose2d();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
