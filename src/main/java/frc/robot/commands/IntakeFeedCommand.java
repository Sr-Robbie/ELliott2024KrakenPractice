package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Feed.Feed;
import frc.robot.subsystems.Feed.States.feedState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.States.intakingState;

public class IntakeFeedCommand extends ParallelCommandGroup { 
    public IntakeFeedCommand(Intake intake, Feed feed, double intakeSpeed, double feedSpeed) {
        super(
            new feedState(feed, feedSpeed), 
            new intakingState(intake, intakeSpeed)
        );
    }
}
