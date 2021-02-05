/*----------------------------------------------------------------------------*/
/* Copyright (c) 2021 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
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
      m_shooter.setShooterPID(0.0005, 0.000000, 0, 0.00018, 250);
      m_shooter.setShooterRPM(2800);

    RobotContainer.m_limelight.setLEDMode(0);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterRPM(0);
    //RobotContainer.m_limelight.setLEDMode(1);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}