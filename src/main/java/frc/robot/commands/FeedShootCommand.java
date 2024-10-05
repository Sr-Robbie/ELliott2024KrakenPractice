// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Feed.Feed;
import frc.robot.subsystems.Feed.States.feedState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.States.shootingState;

public class FeedShootCommand extends ParallelCommandGroup {
  public FeedShootCommand (Shooter shooterMotorTop, Feed feed, double shooterMotorSpeed, double feedSpeed) {
    super (
      new feedState(feed, feedSpeed).beforeStarting(new WaitCommand(.9)), new shootingState(shooterMotorTop, shooterMotorSpeed)
    );
  }
  
}
