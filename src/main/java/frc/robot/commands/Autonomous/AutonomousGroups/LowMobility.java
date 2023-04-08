package frc.robot.commands.Autonomous.AutonomousGroups;
import frc.robot.subsystems.*;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.Autonomous.NonpathUtilityCommands.ManipulatorOutAutonCube;
import frc.robot.commands.Autonomous.RameseteCommands.BackwardAutoConeScore;
import frc.robot.commands.Teleop.AutoArmSetPos;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LowMobility extends SequentialCommandGroup {
    public LowMobility(Drive drive, Arm arm, Manipulator manipulator) {
        addCommands(
            new AutoArmSetPos(ManipulatorConstants.armIntakePoint, arm, true),
            new ManipulatorOutAutonCube(manipulator, 1500),
            new BackwardAutoConeScore(drive).getRamseteCommand()
        );
    }
}