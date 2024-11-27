// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

public CANSparkFlex shooterMotorTop;
public CANSparkFlex shooterMotorBottom;
public void lidar(){};
  public Shooter() {
    shooterMotorTop = new CANSparkFlex(16, MotorType.kBrushless);
     
    shooterMotorTop.setSmartCurrentLimit(40);

    shooterMotorTop.setInverted(false);

    shooterMotorTop.setIdleMode(IdleMode.kBrake);
    //
    shooterMotorBottom = new CANSparkFlex(26, MotorType.kBrushless);

    shooterMotorBottom.setSmartCurrentLimit(40);

    shooterMotorBottom.setInverted(true);

    shooterMotorBottom.setIdleMode(IdleMode.kBrake);

    lidar = new Lidar(11);
  }
  public void setPower(double power) {
    shooterMotorTop.set(power);

    shooterMotorBottom.set(power);
  }

  @Override
  public void periodic() {
  }
}
