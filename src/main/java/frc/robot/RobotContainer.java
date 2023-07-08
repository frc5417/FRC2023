// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.DriverConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autonomous.AutonomousGroups.ConeScoreAuton;
import frc.robot.commands.Autonomous.AutonomousGroups.ConeScoreMoveAuton;
import frc.robot.commands.Autonomous.AutonomousGroups.DockAuton;
import frc.robot.commands.Autonomous.AutonomousGroups.EngageAuton;
import frc.robot.commands.Autonomous.AutonomousGroups.EngageScoreAuton;
import frc.robot.commands.Autonomous.AutonomousGroups.EngageScoreMoveAuton;
import frc.robot.commands.Autonomous.AutonomousGroups.LowMobility;
import frc.robot.commands.Teleop.ArmManualMovement;
import frc.robot.commands.Teleop.ArmSetPos;
import frc.robot.commands.Teleop.AutoBalance;
import frc.robot.commands.Teleop.DriveBreakToggle;
import frc.robot.commands.Teleop.ManipulatorIn;
import frc.robot.commands.Teleop.ManipulatorOut;
import frc.robot.commands.Teleop.SetLightConfig;
import frc.robot.commands.Teleop.TankDrive;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  private static final LightsControl m_lightsControl = new LightsControl();
  private static final Drive m_drive = new Drive();

  private final static Arm armSubsystem = new Arm();
  private final static Manipulator manipulatorSubsystem = new Manipulator();
  
  private static final TankDrive tankDrive = new TankDrive(m_drive);
  private final AutoBalance m_AutoBalance =  new AutoBalance(m_drive);
  private final DriveBreakToggle driveBreak = new DriveBreakToggle(m_drive);
  private final static ArmManualMovement armManualCommand = new ArmManualMovement(armSubsystem);

  private final ConeScoreAuton coneScoreAuton = new ConeScoreAuton(m_drive, armSubsystem, manipulatorSubsystem);
  private final LowMobility lowMobility = new LowMobility(m_drive, armSubsystem, manipulatorSubsystem);
  private final ConeScoreMoveAuton coneScoreMoveAuton = new ConeScoreMoveAuton(m_drive, armSubsystem, manipulatorSubsystem);
  private final DockAuton dockAuton = new DockAuton(m_drive, armSubsystem, manipulatorSubsystem);
  private final EngageAuton engageAuton = new EngageAuton(m_drive, armSubsystem, manipulatorSubsystem);
  private final EngageScoreAuton engageScoreAuton = new EngageScoreAuton(m_drive, armSubsystem, manipulatorSubsystem);
  private final EngageScoreMoveAuton engageScoreMoveAuton = new EngageScoreMoveAuton(m_drive, armSubsystem, manipulatorSubsystem);

  private final static ArmSetPos armSetPointIntake = new ArmSetPos(Constants.ManipulatorConstants.armIntakePoint, armSubsystem);
  private final static ArmSetPos armSetPointSecondScore = new ArmSetPos(Constants.ManipulatorConstants.armSecondScorePoint, armSubsystem);
  private final static ArmSetPos armSetPointThirdScore = new ArmSetPos(Constants.ManipulatorConstants.armThirdScorePoint, armSubsystem);
  private final static ArmSetPos armSetPointHumanCube = new ArmSetPos(Constants.ManipulatorConstants.armHumanCubePoint, armSubsystem);
  
  private final static ManipulatorOut manipulatorOut = new ManipulatorOut(manipulatorSubsystem);
  private final static ManipulatorIn manipulatorIn = new ManipulatorIn(manipulatorSubsystem);
  
  private static final SetLightConfig lightConfigRed = new SetLightConfig(m_lightsControl, 0);
  private static final SetLightConfig lightConfigBlue = new SetLightConfig(m_lightsControl, 4);
  private static final SetLightConfig lightConfigColor1 = new SetLightConfig(m_lightsControl, 1);
  private static final SetLightConfig lightConfigColor2 = new SetLightConfig(m_lightsControl, 2);

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
    m_driverController.a().whileTrue(m_AutoBalance);
    m_driverController.x().toggleOnTrue(driveBreak);
    
    m_manipulatorController.a().whileTrue(armSetPointIntake);
    m_manipulatorController.b().whileTrue(armSetPointSecondScore);
    m_manipulatorController.y().whileTrue(armSetPointThirdScore);
    m_manipulatorController.rightBumper().whileTrue(armSetPointHumanCube) ;

    m_manipulatorController.leftTrigger().whileTrue(manipulatorOut);
    m_manipulatorController.rightTrigger().whileTrue(manipulatorIn);

    m_manipulatorController.povUp().onTrue(lightConfigRed);
    m_manipulatorController.povDown().onTrue(lightConfigBlue);
    m_manipulatorController.povLeft().onTrue(lightConfigColor1);
    m_manipulatorController.povRight().onTrue(lightConfigColor2);

    System.out.println("Buttons Configured");
  }

  public Command coneScoreAutonomousCommand() {
    return coneScoreAuton;
  }

  public Command lowMobilityAutonomouCommand(){
    return lowMobility;
  }

  public Command coneScoreMoveAutonomousCommand() {
    return coneScoreMoveAuton;
  }

  public Command dockAutonomousCommand() {
    return dockAuton;
  }

  public Command engageAutonomousCommand() {
    return engageAuton;
  }

  public Command engageScoreAutonomousCommand() {
    return engageScoreAuton;
  }

  public Command engageScoreMoveAutonomousCommand() {
    return engageScoreMoveAuton;
  }

  public static void setDriverRumble(double rumbleVal) {
    m_driverController.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, rumbleVal);
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

  public static void setBrakeMode() {
    m_drive.setDriveBreak();
  }

  public static void resetOdometry() {
    m_drive.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d()));
  }

  public static void setLEDsOff() {
    m_lightsControl.setLightConfig(3);
  }

  public static void setLEDsOn() {
    m_lightsControl.setLightConfig(0);
  }
}
