// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

  //private final DifferentialDrive diffDrive;

  private final DifferentialDriveOdometry odometry;

  private final DifferentialDriveKinematics kinematics;

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private final double GEAR_RATIO = 8.46;

  private final double velocityConversion = ((GEAR_RATIO) * (Math.PI * Units.inchesToMeters(4)) / 60);

  private final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(6) * Math.PI; 

  private final double KP_DRIVE = 0.00001;
  private final double FF_DRIVE = 0.001;

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
    frontRightConfig.closedLoop.p(KP_DRIVE);
    frontRightConfig.closedLoop.velocityFF(FF_DRIVE);

    frontLeftConfig.inverted(true);
    backLeftConfig.follow(frontLeft);
    frontLeftConfig.closedLoop.p(KP_DRIVE);
    frontLeftConfig.closedLoop.velocityFF(FF_DRIVE);

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

    //diffDrive = new DifferentialDrive(frontLeft, frontRight);

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

    ChassisSpeeds speeds = new ChassisSpeeds();
    speeds.vxMetersPerSecond = xspeed;
    speeds.omegaRadiansPerSecond = zspeed;

    driveRobotRelative(speeds);

    // diffDrive.arcadeDrive(xModified, maxRotation * zspeed);


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

    double leftRotPerMin = wheelSpeeds.leftMetersPerSecond * 60 / WHEEL_CIRCUMFERENCE;
    double rightRotPerMin = wheelSpeeds.rightMetersPerSecond * 60 / WHEEL_CIRCUMFERENCE;

    SmartDashboard.putNumber("LeftM/S", leftRotPerMin);
    SmartDashboard.putNumber("RightM/S", rightRotPerMin);

    frontLeft.getClosedLoopController()
      .setReference(leftRotPerMin, ControlType.kVelocity);

    frontRight.getClosedLoopController()
      .setReference(rightRotPerMin, ControlType.kVelocity);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double leftMotorMeters = (frontLeftEncoder.getPosition() * WHEEL_CIRCUMFERENCE) / GEAR_RATIO;
    double rightMotorMeters = (frontRightEncoder.getPosition() * WHEEL_CIRCUMFERENCE) / GEAR_RATIO;

    odometry.update(
      gyro.getRotation2d(), 
      leftMotorMeters, 
      rightMotorMeters);

    SmartDashboard.putNumber("Gyro", gyro.getAngle());
  }
}
