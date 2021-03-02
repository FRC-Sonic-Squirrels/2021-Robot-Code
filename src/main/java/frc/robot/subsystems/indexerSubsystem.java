/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.indexConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANPIDController;

import static frc.robot.Constants.currentLimits;
import static frc.robot.Constants.digitalIOConstants;

import java.beans.Encoder;

import static frc.robot.Constants.canId;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class indexerSubsystem extends SubsystemBase {

  private WPI_TalonSRX indexIntake;
  private WPI_TalonFX indexKicker;
  private WPI_TalonFX indexBelts;

  private CANSparkMax m_hopperAgitator = new CANSparkMax(indexConstants.hopperAgitator, MotorType.kBrushless);
  private CANPIDController agitatorController = m_hopperAgitator.getPIDController();;
  private DigitalInput Sensor1 = new DigitalInput(digitalIOConstants.dio0_indexerSensor1);
  private DigitalInput Sensor2 = new DigitalInput(digitalIOConstants.dio1_indexerSensor2);
  private DigitalInput Sensor3 = new DigitalInput(digitalIOConstants.dio2_indexerSensor3);
  private boolean ballReady4IndexerLast = false;
  private boolean ballExitingLast = false;
  private boolean ejectMode = false;
  private boolean ejectBallStep1 = false;
  private boolean ejectBallStep2 = false;
  private boolean ejectBallStep3 = false;
  private int ballCount = 0;
  private int restageState = 0;
  // private blinkinSubsystem m_blinkin = RobotContainer.m_blinkin;

  private boolean finishedSingleFeed;

  public indexerSubsystem() {

    indexIntake = new WPI_TalonSRX(canId.canId8_indexo_intake_and_hopper);
    indexBelts = new WPI_TalonFX(canId.canId10_indexo_belts);
    indexKicker = new WPI_TalonFX(canId.canId11_indexo_kicker);
    
    indexBelts.configFactoryDefault();
    indexKicker.configFactoryDefault();
    indexIntake.configFactoryDefault();
    m_hopperAgitator.restoreFactoryDefaults();

    // Voltage limits, percent output is scaled to this new max
    indexBelts.configVoltageCompSaturation(11);
    indexBelts.enableVoltageCompensation(true);
    indexKicker.configVoltageCompSaturation(11);
    indexKicker.enableVoltageCompensation(true);
    indexIntake.configVoltageCompSaturation(11);
    indexIntake.enableVoltageCompensation(true);

    // current limits
    indexBelts.configSupplyCurrentLimit(currentLimits.m_currentlimitSecondary);

    // configSupplyCurrentLimit not available on Victors
    //indexKicker.configSupplyCurrentLimit(currentLimits.m_currentlimitSecondary);
    //indexIntake.configSupplyCurrentLimit(currentLimits.m_currentlimitSecondary);

    // Brake mode
    indexBelts.setNeutralMode(NeutralMode.Brake);
    indexKicker.setNeutralMode(NeutralMode.Brake);
    indexIntake.setNeutralMode(NeutralMode.Brake);

    // Invert
    indexBelts.setInverted(false);
    indexKicker.setInverted(false);
    indexIntake.setInverted(true);

    indexBelts.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    indexKicker.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    indexIntake.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    // TalonFX don't have sensor phase only TalonSRX
    indexIntake.setSensorPhase(false);
    indexKicker.setSensorPhase(false);
    
    //Set Ramp-Up
    //indexKicker.configClosedloopRamp(0.1);
    //indexBelts.configClosedloopRamp(0.1);
    //indexIntake.configClosedloopRamp(0.1);


    // Config PID values to control RPM
    indexBelts.config_kP(0, 0.15, 10);
    indexBelts.config_kI(0, 0.0, 10);
    indexBelts.config_kD(0, 1.5, 10);
    indexBelts.config_kF(0, 0.048, 10);

    indexKicker.config_kP(0, 0.15, 10);
    indexKicker.config_kI(0, 0.0, 10);
    indexKicker.config_kD(0, 1.5, 10);
    indexKicker.config_kF(0, 0.053, 10);

    indexIntake.config_kP(0, 0.1, 10);
    indexIntake.config_kI(0, 0.0, 10);
    indexIntake.config_kD(0, 0.0, 10);
    indexIntake.config_kF(0, 0.0, 10);

    agitatorController.setP(0.1);
    agitatorController.setI(1e-4);
    agitatorController.setD(0);
    agitatorController.setFF(0);
    agitatorController.setIZone(100);
    agitatorController.setOutputRange(-0.1, 0.1);

  }

  @Override
  public void periodic() {
    boolean ballReady4Indexer = true; // !Sensor1.get();
    boolean ballExiting = !Sensor3.get();
    SmartDashboard.putNumber("ball count", ballCount);
    SmartDashboard.putNumber("restage state", restageState);
    SmartDashboard.putNumber("Belt RPM", indexBelts.getSelectedSensorVelocity() * 600 / 2048);
    SmartDashboard.putNumber("Kicker RPM", indexKicker.getSelectedSensorVelocity() * 600 / 2048);

    // eject Mode runs the indexer 7 kicker until one ball has been ejected
    if (ejectMode) {
      if (ejectBallStep3) {
        // step 3, run indexer until next ball is waiting
        SmartDashboard.putNumber("Eject State", 3);
        if (ballExiting) {
          ejectMode = false;
          stopIndexer();
        }
      }
      else if (ejectBallStep2) {
        // step 2, run indexer until ball leaves
        SmartDashboard.putNumber("Eject State", 2);
        if (!ballExiting) {
          ejectBallStep3 = true;
        }
      }
      else if (ejectBallStep1) {
        // Step 1, run indexer until ball is ready to shoot
        SmartDashboard.putNumber("Eject State", 1);
        if (ballExiting) {
            ejectBallStep2 = true;
        }
      }
    }
    else {
      SmartDashboard.putNumber("Eject State", 0);
    }

    // increase ball count as balls enter the indexer
    if (ballReady4Indexer != ballReady4IndexerLast && ballReady4Indexer == false) {
      ballCount += 1;
    }
    ballReady4IndexerLast = ballReady4Indexer;

    // decrease ballCount as balls leave the indexer
    if (ballExiting != ballExitingLast && ballExiting == false) {
      ballCount -= 1;
    }
    ballExitingLast = ballExiting;
  }

  public void setBeltsPercentOutput(double percent) {
    indexBelts.set(ControlMode.PercentOutput, percent);
  }

  public void setKickerPercentOutput(double percent) {
    indexKicker.set(ControlMode.PercentOutput, percent);
  }

  public void setIntakePercentOutput(double percent) {
      indexIntake.set(ControlMode.PercentOutput, percent);
  }

  public void setAgitatorPercentOutput(double percent) {
    m_hopperAgitator.set(percent);
  }

  public void setHopperPercentOutput(double percent){
    setAgitatorPercentOutput(percent);
    setIntakePercentOutput(percent);
  }

  public void setBeltsRPM(double rpm) {
    indexBelts.set(ControlMode.Velocity, rpm * 2048 / 600);
  }

  public void setKickerRPM(double rpm) {
    indexKicker.set(ControlMode.Velocity, rpm * 2048 / 600);
  }

  public void setIntakeRPM(double rpm) {
    indexIntake.set(ControlMode.Velocity, rpm * 2048 / 600);
  }

  public void ejectOneBall() {

    if (ejectMode) {
      // we're already in ejectMode
      return;
    }

    /**
     * steps:
     * 1. run indexer until ball exiting (get ready to shoot)
     * 2. run indexer until ball not exiting (shooting!)
     * 3. stop indexer when ball ready to exit (ready for next shot)
     */
    ejectMode = true;
    ejectBallStep1 = true;
    ejectBallStep2 = ballExiting();
    ejectBallStep3 = false;

    // start the indexer
    ejectIndexer();
  }

  /** 
   * Stop all motors
   */
  public void stopIndexer() {
    ejectMode = false;
    setBeltsPercentOutput(0.0);
    setKickerPercentOutput(0.0);
    setIntakePercentOutput(0.0);
    setAgitatorPercentOutput(0.0);
    // m_blinkin.solid_orange();
  }

  /**
   * ballReadyForIndexer - monitor sensor 1 for a ball ready to be indexed
   * 
   * @return true if a ball is waiting to be indexed
   */
  public boolean ballReadyForIndexer() {
    return true;
    // return ! Sensor1.get();
  }

  /**
   * ballStaged - monitor sensor 2 for a ball that is staged
   * 
   * @return true if a ball is staged
   */
  public boolean ballStaged() {
    return ! Sensor2.get();
  }

  /**
   * ballExiting - monitor sensor 3 for a ball that is at the kickers
   * 
   * @return true if a ball is at the kickers
   */
  public boolean ballExiting() {
    return ! Sensor3.get();
  }

  /**
   * runIndexer() - run all indexer motors at ball staging speeds
   */
  public void runIndexer() {
      setIntakePercentOutput(1);
      setBeltsRPM(6380);
      setKickerPercentOutput(0.3);
      // m_blinkin.solid_green();
  }

  /**
   * runBelts() - run only the belts
   */
  public void runOnlyBelts() {
      setIntakePercentOutput(0);
      setBeltsRPM(6380);
      setKickerPercentOutput(0);
      // m_blinkin.solid_blue();
  }

  /**
   * reverseIndexer() - run all indexer motors backwards at staging speeds
   */
  public void reverseIndexer() {
      setIntakePercentOutput(-0.8);
      setBeltsRPM(-3000);
      setKickerPercentOutput(-0.3);
      // m_blinkin.strobe_red();
  }

  /**
   * ejectIndexer() - run all indexer motors at eject/shooting speeds
   */
  public void ejectIndexer() {
      setIntakePercentOutput(0.8);
      setBeltsRPM(6380);
      setKickerPercentOutput(1);
  }

  /**
   * runIntake() - run intake motor
   */
  public void runIntake() {
    setIntakePercentOutput(1);
  }

  /**
   * stopIntake() - stop intake motor
   */
  public void stopIntake() {
    setIntakePercentOutput(0);
  }

  /**
   * stopHopper() - stop's intake & Intake Agitator Motors
   */
  public void stopHopper() {
    setHopperPercentOutput(0);
  }
  
  /**
   * runOnlyIntake() - run intake motor and stop the belts and kicker
   */
  public void runOnlyIntake() {
      setIntakePercentOutput(1);
      setBeltsRPM(0);
      setKickerPercentOutput(0);
      setAgitatorPercentOutput(0.1);
  } 

  /**
   * stopBelts() - stop the belts motor
   */
  public void stopBelts() {
    setBeltsRPM(0);
  }

  /**
   * stopKicker() - stop the kicker motor
   */
  public void stopKicker() {
    setKickerPercentOutput(0);
  }

  /**
   * getBallCount() - return the number of balls in the indexer
   * 
   * @return int ball count
   */
  public int getBallCount() {
    return ballCount;
  }

  /**
   * setBallCount() - set the number of balls in the indexer
   */
  public void setBallCount(int BallCount) {
    ballCount = BallCount;
  }

  /**
   * getRestageState() - return the state number of the restage command
   * 
   * @return int restage state
   */
  public int getRestageState() {
    return restageState;
  }

  /**
   * setRestageState() - set the restage state
   */
  public void setRestageState(int RestageState) {
    restageState = RestageState;
  }

  /**
   * setFinishedSingleFeed() - set the status of the single feed command
   */
  public void setFinishedSingleFeed(boolean finished) {
    finishedSingleFeed = finished;
  }

  /**
   * getFinishedSingleFeed() - gets the status of the single feed command
   * 
   * @return boolean finished single feed
   */
  public boolean getFinishedSingleFeed() {
    return finishedSingleFeed;
  }
}
