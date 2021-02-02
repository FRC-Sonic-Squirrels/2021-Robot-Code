/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.driveSubsystem;

public class driveCommand extends CommandBase {

  driveSubsystem m_drive;
  XboxController driveController = RobotContainer.m_driveController;

  public driveCommand(driveSubsystem drive) {
    addRequirements(drive);
    m_drive = drive;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (m_drive.getDriveInvert() == false) {
      if (driveController.getBumper(Hand.kLeft) == false) {
        m_drive.arcadeDrive(driveController.getY(Hand.kLeft) * 0.5, -driveController.getX(Hand.kRight) * 0.5);  
      }
      else {
        m_drive.arcadeDrive(driveController.getY(Hand.kLeft), -driveController.getX(Hand.kRight));
      }
    }

    if (m_drive.getDriveInvert() == true) {
      if (driveController.getBumper(Hand.kLeft) == false) {
        m_drive.arcadeDrive(-driveController.getY(Hand.kLeft) * 0.5, -driveController.getX(Hand.kRight) * 0.5);
      }
      else {
        m_drive.arcadeDrive(-driveController.getY(Hand.kLeft), -driveController.getX(Hand.kRight));
      }
    }
    }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}