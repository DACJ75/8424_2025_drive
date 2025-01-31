// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private final double velocityConversion = (8.45) * (Math.PI * Units.inchesToMeters(4)) / 60;

  private final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(6) * Math.PI; 

  RobotConfig config;

  public DriveSubsystem() {
    frontRight = new SparkMax(1, MotorType.kBrushless);
    backRight = new SparkMax(2, MotorType.kBrushless);
    frontLeft = new SparkMax(3, MotorType.kBrushless);
    backLeft = new SparkMax(4, MotorType.kBrushless);
    

    frontRightEncoder = frontRight.getEncoder();
    frontLeftEncoder = frontLeft.getEncoder();

    backRightConfig = new SparkMaxConfig();
    backLeftConfig = new SparkMaxConfig();
    frontRightConfig = new SparkMaxConfig();
    frontLeftConfig = new SparkMaxConfig();
    
    
    frontRightConfig.inverted(false);
    backRightConfig.follow(frontRight);

    frontLeftConfig.inverted(true);
    backLeftConfig.follow(frontLeft);

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

    try{
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds,feedforwards) ->driveRobotRelative(speeds),
        new PPLTVController(0.02),
        config,
        () -> {

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) { 
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this
      );
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  //This is massive. do you know what else is massive. LOOWWWWWW TAPER FADE!!!!!!! ITs just so massive
  public void drive(double xspeed, double zspeed) {

    double maxSpeed = 0.6;
    double maxRotation = 0.6;
    double xModified = maxSpeed * Math.pow(xspeed, 3);

    diffDrive.arcadeDrive(xModified, maxRotation * zspeed);
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
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    frontLeft.getClosedLoopController()
      .setReference(wheelSpeeds.leftMetersPerSecond * 60 / WHEEL_CIRCUMFERENCE, ControlType.kMAXMotionVelocityControl);
    frontRight.getClosedLoopController()
      .setReference(wheelSpeeds.rightMetersPerSecond * 60 / WHEEL_CIRCUMFERENCE, ControlType.kMAXMotionVelocityControl);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(
      gyro.getRotation2d(), 
      frontLeftEncoder.getPosition(), 
      frontRightEncoder.getPosition());

    SmartDashboard.putNumber("Gyro", gyro.getAngle());
  }
}
