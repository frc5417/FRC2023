package frc.robot.commands;
import frc.robot.subsystems.*;
import frc.robot.Constants.ManipulatorConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class EngageAuton extends SequentialCommandGroup {
    public EngageAuton(Drive drive, Arm arm, Manipulator manipulator) {
        addCommands(
            new AutoArmSetPos(ManipulatorConstants.armIntakePoint, arm, true),
            new ManipulatorOutAutonCube(manipulator, 300),
            new BackwardEngageAutoConeScore(drive).getRamseteCommand(),
            new AutoBalance(drive)
        );
    }
}