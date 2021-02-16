// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intakeSubsystem extends SubsystemBase {

  WPI_TalonFX m_intake = new WPI_TalonFX(Constants.intakeConstants.intakeMotor);

  public intakeSubsystem() { 
  }

  @Override
  public void periodic() {
  }

  public void setIntakePercentOutput(double percent){
      m_intake.set(ControlMode.PercentOutput, percent);
  }
}
