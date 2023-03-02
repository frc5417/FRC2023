// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.photonvision.PhotonCamera;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final PhotonSubsystem m_photonsubsystem = new PhotonSubsystem();
  // private final PhotonCommand m_pPhotonCommand = new PhotonCommand(m_photonsubsystem);

  public static final AHRS ahrs = new AHRS(SerialPort.Port.kMXP); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */

  private static final PhotonSubsystem m_photonsubsystem = new PhotonSubsystem();
  private static final Drive m_drive = new Drive();
  private static final NavXGyro m_NavXGyro = new NavXGyro();
  private static final LightsControl m_lightsControl = new LightsControl();

  private final PhotonDirectDrive m_photonDirectDriveCommand = new PhotonDirectDrive(m_photonsubsystem, m_drive);
  private static final PhotonCommand m_pPhotonCommand = new PhotonCommand(m_photonsubsystem);
  private static final NavXGyroCommand m_NavXGyroCommand = new NavXGyroCommand(m_NavXGyro, ahrs, m_drive, m_photonsubsystem);
  private static TankDrive tankDrive = new TankDrive(m_drive, ahrs, m_pPhotonCommand, m_NavXGyroCommand);
  private static final SetLightConfig lightConfigOff = new SetLightConfig(m_lightsControl, 0);
  private static final SetLightConfig lightConfigColor1 = new SetLightConfig(m_lightsControl, 1);
  private static final SetLightConfig lightConfigColor2 = new SetLightConfig(m_lightsControl, 2);

  private final PhotonCamera camera = m_photonsubsystem.PhotonCameraWrapper();


  
  
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static final CommandXboxController m_driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

    private static final CommandXboxController m_manipulatorController =
      new CommandXboxController(1);

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
    m_driverController.rightBumper().whileTrue(m_photonDirectDriveCommand);

    m_manipulatorController.povUp().onTrue(lightConfigOff);
    m_manipulatorController.povUpLeft().onTrue(lightConfigColor1);
    m_manipulatorController.povUpRight().onTrue(lightConfigColor2);
  }

  public static CommandXboxController getDriverController() {
    return m_driverController;
  }
  public static double getDriverLeftJoystick() {
    return m_driverController.getRawAxis(1);
  }

  public static boolean getButtonA() {
    return m_driverController.a().getAsBoolean();
  }
  
  public static boolean getRtBumper() {
    return m_driverController.rightBumper().getAsBoolean();
  }

  public static boolean getButtonX() {
    return m_driverController.x().getAsBoolean();
  }

  public static double getDriverRightJoystick() {
    return m_driverController.getRawAxis(5);
  }
  
  public static void initTelopCommands() {
    tankDrive.schedule();
  }

  public static void initTankDrive() {
    tankDrive.schedule();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void getAutonomousCommand() {
    // An example command will be run in autonomous
    // return m;
  }

  public static void turnLightColor1() {
    lightConfigColor1.schedule();
  }

  public static void turnLightColor2() {
    lightConfigColor2.schedule();
  }
}