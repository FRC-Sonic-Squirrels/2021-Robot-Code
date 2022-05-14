
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turretSubsystem;

public class turretManualMode extends CommandBase {
  
  private turretSubsystem m_turret;
  XboxController controller = RobotContainer.m_operatorController;
  XboxController driver = RobotContainer.m_driveController;

  public turretManualMode(turretSubsystem turret) {
    addRequirements(turret);
    m_turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("turretManualCommand", true);
    double stickInput = driver.getRightX();
    if (Math.abs(stickInput) > 0.5) {
      // invert stick, as positive is CCW
      m_turret.setPercentOutput( - stickInput * 0.1);
    }

    else {
      m_turret.setPercentOutput(0.0);

      if (controller.getLeftStickButtonPressed()) {
        m_turret.setAngleDegrees(0);  // home
      }
      if (controller.getXButton()) {
        m_turret.setAngleDegrees(10);
      }
      if (controller.getYButton()) {
        m_turret.setAngleDegrees(-10);
      }
      if (controller.getLeftBumperPressed()) {
        double currentAngleDegrees = m_turret.getAngleDegrees();
        // CCW is positive
        m_turret.setAngleDegrees(currentAngleDegrees + 5);
      }
      if (controller.getRightBumperPressed()) {
        double currentAngleDegrees = m_turret.getAngleDegrees();
        // CW is negative
        m_turret.setAngleDegrees(currentAngleDegrees - 5);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
    SmartDashboard.putBoolean("turretManualCommand", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
