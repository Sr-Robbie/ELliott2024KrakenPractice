// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;

public class shootingState extends Command {
  Shooter shooterMotorTop;
  double power;
  public shootingState(Shooter shooterMotorTop, double power) {
    this.shooterMotorTop = shooterMotorTop;
    this.power = power;
    addRequirements(shooterMotorTop);
  }

  @Override
  public void initialize() {
    shooterMotorTop.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    shooterMotorTop.setPower(0.0);
  }
}