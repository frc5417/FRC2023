// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.TimeUnit.*;

import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.DriverConstants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final Drive m_drive = new Drive();
  private static final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  private final static Arm armSubsystem = new Arm();
  private final static Manipulator manipulatorSubsystem = new Manipulator();
  private static final TankDrive tankDrive = new TankDrive(m_drive);
  private final AutoBalance m_AutoBalance =  new AutoBalance(m_drive);
  private final DriveBreakToggle driveBreak = new DriveBreakToggle(m_drive);
  private static final ShiftDrivetrain shiftDrivetrain = new ShiftDrivetrain(m_drive);
  private final static ArmManualMovement armManualCommand = new ArmManualMovement(armSubsystem);

  private final static ArmSetPos armSetPointIntake = new ArmSetPos(0.998, armSubsystem);
  private final static ArmSetPos armSetPointSecondScore = new ArmSetPos(0.787, armSubsystem);
  private final static ArmSetPos armSetPointThirdScore = new ArmSetPos(0.720, armSubsystem);
  private final static ArmSetPos armSetPointHumanCone = new ArmSetPos(0.759, armSubsystem);
  private final static ArmSetPos armSetPointHumanCube = new ArmSetPos(0.767, armSubsystem);
  private final static ManipulatorOut manipulatorOut = new ManipulatorOut(manipulatorSubsystem);
  private final static ManipulatorOutAuton manipulatorOutAuton1 = new ManipulatorOutAuton(manipulatorSubsystem, 750);
  private final static ManipulatorIn manipulatorIn = new ManipulatorIn(manipulatorSubsystem);
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
    m_driverController.a().whileTrue(m_AutoBalance);
    m_driverController.x().toggleOnTrue(driveBreak);
    m_driverController.b().whileTrue(shiftDrivetrain);
    
    m_manipulatorController.a().whileTrue(armSetPointIntake);
    m_manipulatorController.b().whileTrue(armSetPointSecondScore);
    m_manipulatorController.y().whileTrue(armSetPointHumanCube) ;
    m_manipulatorController.x().whileTrue(armSetPointHumanCone);
    m_manipulatorController.rightBumper().whileTrue(armSetPointThirdScore);

    m_manipulatorController.povUp().onTrue(clawConfig1);
    m_manipulatorController.povRight().onTrue(clawConfig2);
    m_manipulatorController.povDown().onTrue(clawConfig3);

    m_manipulatorController.leftTrigger().whileTrue(manipulatorOut);//.onFalse(manipulatorSpeedOff);
    m_manipulatorController.rightTrigger().whileTrue(manipulatorIn);//.onFalse(manipulatorSpeedOff);

    System.out.println("Buttons Configured");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static void setDriverRumble(double rumbleVal) {
    m_driverController.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, rumbleVal);
    m_driverController.getHID().setRumble(GenericHID.RumbleType.kRightRumble, rumbleVal);
  }

  public static double getDriverLeftJoystick() {
    double value = m_driverController.getRawAxis(1);

    if(Math.abs(value) < ManipulatorConstants.kManipulatorControllerDeadZone) {
      value = 0;
    }

    return value;
  }

  public static double getDriverRightJoystick() {
    double value = m_driverController.getRawAxis(5);

    if(Math.abs(value) < ManipulatorConstants.kManipulatorControllerDeadZone) {
      value = 0;
    }

    return value;
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

  /*public Command getAutonomousCommand() {
    
  }*/

  public static void initArmMovement() {
    armManualCommand.schedule();
  }

  public static void initTeleopCommand(){
    m_drive.setDriveCoast();
    tankDrive.schedule();
  } 

  public static void setCoastMode() {
    m_drive.setDriveCoast();
  }
}
