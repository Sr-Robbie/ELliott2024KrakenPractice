// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

public CANSparkFlex intakeMotor;

  public Intake() {
    intakeMotor = new CANSparkFlex(15, MotorType.kBrushless);

    intakeMotor.setSmartCurrentLimit(80);
  
    intakeMotor.setInverted(false);

    intakeMotor.setIdleMode(IdleMode.kCoast);
  
    }

  public void setPower(double power) {
    intakeMotor.set(power);
  }


  @Override
  public void periodic() {
  
  }
}
