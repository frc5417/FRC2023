// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
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
  private final static Arm armSubsystem = new Arm();
  private final static Manipulator manipulatorSubsystem = new Manipulator();
  private static final Drive m_drive = new Drive();

  private final static ArmManualMovement armManualCommand = new ArmManualMovement(armSubsystem);
  // private final static TankDrive tankDrive = new TankDrive(m_drive);
  private final static ArmSetPos armSetPos7 = new ArmSetPos(0.7d, armSubsystem);
  private final static ArmSetPos armSetPos75 = new ArmSetPos(0.75d, armSubsystem);
  private final static ArmSetPos armSetPos8 = new ArmSetPos(0.8d, armSubsystem);
  private final static ArmSetPos armSetPos85 = new ArmSetPos(0.85d, armSubsystem);
  private final static ArmSetPos armSetPos9 = new ArmSetPos(0.9d, armSubsystem);
  private final static ManipulatorInOpen manipulatorInOpen = new ManipulatorInOpen(manipulatorSubsystem);
  private final static ManipulatorInClosed manipulatorInClosed = new ManipulatorInClosed(manipulatorSubsystem);
  private final static ManipulatorOffOpen manipulatorOffOpen = new ManipulatorOffOpen(manipulatorSubsystem);
  private final static ManipulatorOffClosed manipulatorOffClosed = new ManipulatorOffClosed(manipulatorSubsystem);
  private final static ManipulatorOutOpen manipulatorOutOpen = new ManipulatorOutOpen(manipulatorSubsystem);
  private final static ManipulatorOutClosed manipulatorOutClosed = new ManipulatorOutClosed(manipulatorSubsystem);
  private static final TankDrive tankDrive = new TankDrive(m_drive);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static CommandXboxController m_driverController =
      new CommandXboxController(DriverConstants.kDriverControllerPort);
  private final static CommandXboxController m_manipulatorController =
      new CommandXboxController(ManipulatorConstants.kManipulatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    m_manipulatorController.a().whileTrue(armSetPos9);
    m_manipulatorController.b().whileTrue(armSetPos85);
    m_manipulatorController.y().whileTrue(armSetPos8);
    m_manipulatorController.x().whileTrue(armSetPos75);
    m_manipulatorController.rightBumper().whileTrue(armSetPos7);

    m_manipulatorController.leftTrigger().whileTrue(manipulatorInOpen);
    m_manipulatorController.rightTrigger().whileTrue(manipulatorOffClosed);

    //testing for removal of objects in the intake:
    //m_manipulatorController.leftTrigger().whileTrue(manipulatorInClosed);
    //m_manipulatorController.rightTrigger().whileTrue(manipulatorOutClosed);

    System.out.println("Buttons Configured");
  }

  public static double getDriverLeftJoystick() {
    return m_driverController.getRawAxis(1);
  }

  public static double getDriverRightJoystick() {
    return m_driverController.getRawAxis(5);
  }

  public static double getManipulatorLeftJoystick() {
    double value = m_manipulatorController.getRawAxis(1);

    if(Math.abs(value) < ManipulatorConstants.kManipulatorControllerDeadZone) {
      value = 0;
    }

    return value;
  }
  
  public static double getManipulatorRightJoystick() {
    double value = m_manipulatorController.getRawAxis(5);

    if(Math.abs(value) < ManipulatorConstants.kManipulatorControllerDeadZone) {
      value = 0;
    }

    return value;
  }

  public static void initArmMovement() {
    armManualCommand.schedule();
  }

  public static void initTeleopCommand(){
    tankDrive.schedule();
  }
}
