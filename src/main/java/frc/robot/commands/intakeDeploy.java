// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.intakeSubsystem;

public class intakeDeploy extends CommandBase {
  private intakeSubsystem m_intake;
  private indexerSubsystem m_indexer;
  private long m_delayMs;
  private long start_time = 0;

  /** Creates a new intakeDeploy. */
  public intakeDeploy(intakeSubsystem intake, indexerSubsystem indexer, long motorDelayMs) {
    m_intake = intake;
    m_indexer = indexer;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.releaseIntake();        // open solenoids
    m_intake.deployIntake();         // deploy with pneumatics
    m_indexer.setIntakeMode();       // indexer intake mode

    start_time = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // delay starting the intake until it clears the robot.
    if ((start_time  > 0) && (System.currentTimeMillis() >= start_time + m_delayMs)) {
      m_intake.setDynamicSpeed(true);  // start motors
      start_time = 0;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();           // stop motors
    m_intake.retractIntake();  // retract with pneumatics
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
