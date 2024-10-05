// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Feed;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Feed extends SubsystemBase {
    public CANSparkMax feedMotor;
  public Feed() {
    feedMotor = new CANSparkMax(17, MotorType.kBrushless);
  
    feedMotor.setSmartCurrentLimit(40);

    feedMotor.setInverted(false);

    feedMotor.setIdleMode(IdleMode.kCoast);

  }
      public void setPower(double power) {
        feedMotor.set(power);
  
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
