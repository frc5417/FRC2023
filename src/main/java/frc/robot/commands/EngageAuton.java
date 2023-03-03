package frc.robot.commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class EngageAuton extends SequentialCommandGroup {
    public EngageAuton(Drive drive, Arm arm, Manipulator manipulator) {
        addCommands(
            new AutoShiftDown(drive),
            new AutoArmSetPos(0.80, arm, true),
            new ManipulatorOutAutonCube(manipulator, 375),
            new BackwardEngageAutoConeScore(drive).getRamseteCommand(),
            new AutoBalance(drive)
        );
    }
}