// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class intakingState extends Command {
  /** Creates a new intakingState. */
    Intake intake;
    double power;

  public intakingState(Intake intake, double power) {
    this.intake = intake;
    addRequirements(intake);
  }

  
  @Override
  public void initialize() {
    intake.setPower(power);
  }

  
  @Override
  public void end(boolean interrupted) {
    intake.setPower(0.0);
  }


}
