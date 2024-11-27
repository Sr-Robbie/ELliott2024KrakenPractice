// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Feed.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feed.Feed;

public class feedState extends Command {
  /** Creates a new feedStates. */
  Feed feed;
  double power; //may need to change 'power' to a different word?
  public feedState(Feed feed, double power) {
    this .feed = feed;
    this.power = power;
    addRequirements(feed);
  }

  @Override
  public void initialize() {
    feed.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    feed.setPower(0.0);
  }

  @Override
  public boolean isFinished() {
      // TODO Auto-generated method stub
      return super.isFinished();
  }

}
