// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.mathExtras;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AdvancedSwerveDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Autons.TestAuton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LinearSlideSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final LinearSlideSubsystem m_linearSlideSubsystem = new LinearSlideSubsystem();
  private final LiftSubsystem m_liftSubsystem = new LiftSubsystem();

  private final SendableChooser<Command> autoChooser;

  double deadband = 0.2;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_visionSubsystem.turnTowardAprilTag(m_swerveSubsystem).addRequirements(m_swerveSubsystem);

    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveCommand(
      ()-> mathExtras.deadband(-m_driverController.getLeftY(), deadband),
      ()-> mathExtras.deadband(-m_driverController.getLeftX(), deadband),
      ()-> mathExtras.deadband(-m_driverController.getRightX(), deadband),
      ()-> mathExtras.deadband(-m_driverController.getRightY(), deadband)));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("TestAuton", new TestAuton(m_swerveSubsystem));

    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    SmartDashboard.putNumber("LeftY ", mathExtras.deadband(-m_driverController.getLeftY(), deadband));
    SmartDashboard.putNumber("LeftX ", mathExtras.deadband(-m_driverController.getLeftX(), deadband));
    SmartDashboard.putNumber("RightY ", mathExtras.deadband(-m_driverController.getRightY(), deadband));
    SmartDashboard.putNumber("RightX ", mathExtras.deadband(-m_driverController.getRightX(), deadband));

    SmartDashboard.putNumber("Limelight TX ", m_visionSubsystem.getTx());

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    m_driverController.a().whileTrue(new AdvancedSwerveDriveCommand(m_swerveSubsystem));
    m_driverController.b().whileTrue(
      new AdvancedSwerveDriveCommand(
        m_swerveSubsystem,
         ()-> m_driverController.getLeftY(),
         ()-> m_driverController.getLeftX(),
         ()-> m_driverController.getRightX()));
    m_driverController.x().whileTrue(
      new AdvancedSwerveDriveCommand(
        m_swerveSubsystem,
        0.25,
        90,
        1,
        360,
        ()-> m_swerveSubsystem.getCurrentAngle(),
        m_swerveSubsystem.getCurrentAngle()));
    m_driverController.y().whileTrue(m_visionSubsystem.moveAndAlignTowardAprilTag(m_swerveSubsystem));
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
