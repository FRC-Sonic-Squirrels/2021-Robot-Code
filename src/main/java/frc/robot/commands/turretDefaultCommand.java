/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turretSubsystem;

public class turretDefaultCommand extends CommandBase {
  
  turretSubsystem m_turret;
  XboxController opController = RobotContainer.m_operatorController;

  public turretDefaultCommand(turretSubsystem turret) {
    addRequirements(turret);
    m_turret = turret;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (Math.abs(opController.getLeftX()) >= 0.1) {
      m_turret.setPercentOutput(opController.getLeftX() * 0.5);
    }
    else {
      m_turret.setPercentOutput(0);
    }

    // The start button is used in RobotContainer.java
    // if (opController.getStartButton() == true) {
    //   m_turret.turretHome();
    // }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
