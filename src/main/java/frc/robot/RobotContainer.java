// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.*;
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

  private static final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  private final static ArmManualMovement armManualCommand = new ArmManualMovement(armSubsystem);
  // private final static TankDrive tankDrive = new TankDrive(m_drive);
  private final static ArmSetPos armSetPos3 = new ArmSetPos(0.3d, armSubsystem);
  private final static ArmSetPos armSetPos25 = new ArmSetPos(0.25d, armSubsystem);
  private final static ArmSetPos armSetPos2 = new ArmSetPos(0.2d, armSubsystem);
  private final static ArmSetPos armSetPos15 = new ArmSetPos(0.15d, armSubsystem);
  private final static ArmSetPos armSetPos1 = new ArmSetPos(0.1d, armSubsystem);
  private final static ManipulatorIn manipulatorIn = new ManipulatorIn(manipulatorSubsystem);
  private final static ManipulatorOut manipulatorOut = new ManipulatorOut(manipulatorSubsystem);
  private final static ManipulatorSpeedOff manipulatorSpeedOff = new ManipulatorSpeedOff(manipulatorSubsystem);
  private final static SolenoidClaw clawConfig1 = new SolenoidClaw(1, manipulatorSubsystem);
  private final static SolenoidClaw clawConfig2 = new SolenoidClaw(2, manipulatorSubsystem);
  private final static SolenoidClaw clawConfig3 = new SolenoidClaw(3, manipulatorSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static CommandXboxController m_driverController =
      new CommandXboxController(DriverConstants.kDriverControllerPort);
  private final static CommandXboxController m_manipulatorController =
      new CommandXboxController(ManipulatorConstants.kManipulatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    compressor.enableDigital();
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
    
    m_manipulatorController.a().whileTrue(armSetPos3);
    m_manipulatorController.b().whileTrue(armSetPos25);
    m_manipulatorController.y().whileTrue(armSetPos2);
    m_manipulatorController.x().whileTrue(armSetPos15);
    m_manipulatorController.rightBumper().whileTrue(armSetPos1);

    m_manipulatorController.povUp().onTrue(clawConfig1);
    m_manipulatorController.povRight().onTrue(clawConfig2);
    m_manipulatorController.povDown().onTrue(clawConfig3);

    m_manipulatorController.leftTrigger().onTrue(manipulatorIn).onFalse(manipulatorSpeedOff);
    m_manipulatorController.rightTrigger().onTrue(manipulatorOut).onFalse(manipulatorSpeedOff);

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
}
