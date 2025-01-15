// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveDistanceCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final double m_distance;

  /** Creates a new DrivePosCommand. */
  public DriveDistanceCommand(DriveSubsystem driveSubsystem, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_distance = distance;

    addRequirements(m_driveSubsystem);

    driveSubsystem.resetEncoders();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(0.5, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_driveSubsystem.getEncoderPosition(m_driveSubsystem.frontLeft.getEncoder()) > m_distance)
      return true;
    else
      return false;
  }
}
