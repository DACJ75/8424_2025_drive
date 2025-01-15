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
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
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

  private final DifferentialDriveOdometry odometry;

  private final DifferentialDriveKinematics kinematics;

  private final AHRS gyro = new AHRS(NavXComType.kI2C);

  private final Double velocityConversion = (8.45) * (Math.PI * Units.inchesToMeters(4)) / 60;

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

    backRight.configure(
      backRightConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);

    frontRight.configure(
      frontRightConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);

    frontLeft.configure(
      frontLeftConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);

    diffDrive = new DifferentialDrive(frontLeft, frontRight);

    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22));

    odometry = new DifferentialDriveOdometry(
      gyro.getRotation2d(), 
      frontLeftEncoder.getPosition(), 
      frontRightEncoder.getPosition());
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
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPose(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(
        frontLeftEncoder.getVelocity() * velocityConversion, 
        frontRightEncoder.getVelocity() * velocityConversion));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(
      gyro.getRotation2d(), 
      frontLeftEncoder.getPosition(), 
      frontRightEncoder.getPosition());
  }
}
