// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  //private final ArmSubsystem m_armSubsytem = new ArmSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandJoystick m_driverController = new CommandJoystick(
      OperatorConstants.kDriverControllerPort);

  public final CommandJoystick nextGenJoystick = new CommandJoystick(1);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   * @return
   */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    // Configure the trigger bindings
    configureBindings();
    autoChooser.setDefaultOption("Middle2", getAutonomousCommand());
    SmartDashboard.putData("Auto Choices", autoChooser);
  }

    // {PathPlannerPath path = PathPlannerPath.fromPathFile("Middle 2");
    // Object drivetrain;
    // Command command = ((PathPlannerAuto) drivetrain).PathPlannerAuto(path);
    // return new AutoRoutine("Middle 2", command, List.of(path),
    // path.getStartingDifferentialPose());
  
  // }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release

    m_driveSubsystem.setDefaultCommand(
        new DriveCommand(
             m_driveSubsystem(
            () -> m_driverController.getLeftY() + nextGenJoystick.getRawAxis(1), m_driveSubsystem)));
            () -> m_driverController.getRightX() + nextGenJoystick.getRawAxis(2), m_driveSubsystem;
          
    DriveSubsystem.setDefaultCommand(new DriveCommand(
   () -> -m_driverController.getLeftY() *
    (m_driverController.getHID())
   
            

    // m_driverController.leftTrigger().onTrue(Commands.runOnce(() -> m_armSubsytem.moveArmBackward(), m_armSubsytem));
    // m_driverController.rightTrigger().onTrue(Commands.runOnce(() -> m_armSubsytem.moveArmForward(), m_armSubsytem));
    // m_driverController.rightBumper().onTrue(Commands.runOnce(() -> m_armSubsytem.moveWristForward(), m_armSubsytem));
    // m_driverController.leftBumper().onTrue(Commands.runOnce(() -> m_armSubsytem.moveWristBackward(), m_armSubsytem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  
  }

}
