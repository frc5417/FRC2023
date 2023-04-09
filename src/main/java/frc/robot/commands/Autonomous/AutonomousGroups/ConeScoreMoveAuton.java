package frc.robot.commands.Autonomous.AutonomousGroups;
import frc.robot.subsystems.*;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.Autonomous.NonpathUtilityCommands.ManipulatorOutAuton;
import frc.robot.commands.Autonomous.NonpathUtilityCommands.ManipulatorOutAutonCube;
import frc.robot.commands.Autonomous.RameseteCommands.BackwardAutoConeScore2;
import frc.robot.commands.Autonomous.RameseteCommands.BackwardAutoConeScore3;
import frc.robot.commands.Autonomous.RameseteCommands.ForwardAutoConeScore;
import frc.robot.commands.Teleop.AutoArmSetPos;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeScoreMoveAuton extends SequentialCommandGroup {
    public ConeScoreMoveAuton(Drive drive, Arm arm, Manipulator manipulator) {
        addCommands(
            new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, true),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new ForwardAutoConeScore(drive).getRamseteCommand()
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new ManipulatorOutAutonCube(manipulator, 500)
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new BackwardAutoConeScore2(drive).getRamseteCommand()
            ),
            new AutoArmSetPos(ManipulatorConstants.armIntakePoint, arm, true),
            new BackwardAutoConeScore3(drive).getRamseteCommand(),
            new ManipulatorOutAuton(manipulator, 500)
        );
    }
}
