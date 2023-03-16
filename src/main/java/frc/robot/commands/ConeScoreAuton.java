package frc.robot.commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeScoreAuton extends SequentialCommandGroup {
    public ConeScoreAuton(Drive drive, Arm arm, Manipulator manipulator) {
        addCommands(
            new AutoArmSetPos(0.720, arm, true),
            new ParallelRaceGroup(
                new AutoArmSetPos(0.720, arm, false),
                new ForwardAutoConeScore(drive).getRamseteCommand()
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(0.720, arm, false),
                new ManipulatorOutAuton(manipulator, 2000)
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(0.720, arm, false),
                new BackwardAutoConeScore(drive).getRamseteCommand()
            ),
            new AutoArmSetPos(0.998, arm, true)
        );
    }
}
