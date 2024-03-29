package frc.robot.commands.Autonomous.AutonomousGroups;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.Autonomous.NonpathUtilityCommands.ManipulatorOutAutonCube;
import frc.robot.commands.Autonomous.RameseteCommands.BackwardEngageAutoConeScore2;
import frc.robot.commands.Autonomous.RameseteCommands.BackwardEngageAutoConeScore4;
import frc.robot.commands.Autonomous.RameseteCommands.ForwardEngageAutoConeScore;
import frc.robot.commands.Autonomous.RameseteCommands.ForwardEngageAutoConeScore2;
import frc.robot.commands.Teleop.AutoArmSetPos;
import frc.robot.commands.Teleop.AutoBalance;
import frc.robot.commands.Teleop.DriveBreakAuton;
import frc.robot.commands.Teleop.DriveCoastAuton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class EngageScoreMoveAuton extends SequentialCommandGroup {
    public EngageScoreMoveAuton(Drive drive, Arm arm, Manipulator manipulator) {
        addCommands(
            new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, true),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                //TODO - increase how much it goes forward before competition
                new ForwardEngageAutoConeScore(drive).getRamseteCommand()
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new ManipulatorOutAutonCube(manipulator, 500)
            ),
            new WaitCommand(3.0),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new BackwardEngageAutoConeScore2(drive).getRamseteCommand()
            ),
            new DriveBreakAuton(drive),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armIntakePoint, arm, false),
                new BackwardEngageAutoConeScore4(drive).getRamseteCommand()
            ),
            new DriveCoastAuton(drive),
            new ForwardEngageAutoConeScore2(drive).getRamseteCommand(),
            new AutoBalance(drive)
        );
    }
}