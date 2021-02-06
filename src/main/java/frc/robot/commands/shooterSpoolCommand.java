/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooterSubsystem;

public class shooterSpoolCommand extends CommandBase {
  
  shooterSubsystem m_shooter;
  
  public shooterSpoolCommand(shooterSubsystem shooter) {
    addRequirements(shooter);
    m_shooter = shooter;
  }

  @Override
  public void initialize() {
    double setpoint = m_shooter.getSetPoint();
    if (setpoint < 2800) {
      m_shooter.setShooterRPM(2800);
    }
    else {
      m_shooter.setShooterRPM(0);
    }
    
    RobotContainer.m_limelight.setLEDMode(0);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
