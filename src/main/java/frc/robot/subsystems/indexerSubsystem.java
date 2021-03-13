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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.indexConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;

import static frc.robot.Constants.currentLimits;
import static frc.robot.Constants.digitalIOConstants;
import static frc.robot.Constants.canId;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class indexerSubsystem extends SubsystemBase {

  enum Mode {
    INTAKE,
    EJECT,
    REVERSE,
    STOP
  };

  private WPI_TalonSRX indexIntake;
  private WPI_TalonFX indexKicker;
  private WPI_TalonFX indexBelts;

  private CANSparkMax m_hopperAgitator;
  private CANEncoder m_agitator_encoder;
  private CANPIDController agitatorController;
  private DigitalInput Sensor1 = new DigitalInput(digitalIOConstants.dio0_indexerSensor1);
  private DigitalInput Sensor2 = new DigitalInput(digitalIOConstants.dio1_indexerSensor2);
  private DigitalInput Sensor3 = new DigitalInput(digitalIOConstants.dio2_indexerSensor3);
  private boolean ballReady4IndexerLast = false;
  private boolean ballExitingLast = false;
  private boolean ejectBallStep1 = false;
  private boolean ejectBallStep2 = false;
  private boolean ejectBallStep3 = false;
  private int ballCount = 0;
  private Mode mode = Mode.STOP;
  // private blinkinSubsystem m_blinkin = RobotContainer.m_blinkin;
  

  public indexerSubsystem() {

    indexIntake = new WPI_TalonSRX(canId.canId8_indexo_intake_and_hopper);
    indexBelts = new WPI_TalonFX(canId.canId10_indexo_belts);
    indexKicker = new WPI_TalonFX(canId.canId11_indexo_kicker);

    m_hopperAgitator = new CANSparkMax(indexConstants.hopperAgitator, MotorType.kBrushless);
    m_hopperAgitator.restoreFactoryDefaults();
    agitatorController = m_hopperAgitator.getPIDController();
    m_agitator_encoder = m_hopperAgitator.getEncoder();
    m_hopperAgitator.setInverted(true);

    indexBelts.configFactoryDefault();
    indexKicker.configFactoryDefault();
    indexIntake.configFactoryDefault();

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

    agitatorController.setP(0.00003);
    agitatorController.setI(0.0);
    agitatorController.setD(0);
    agitatorController.setFF(0.00012);
    agitatorController.setIZone(10);
    agitatorController.setOutputRange(-0.8, 0.8);

  }

  @Override
  public void periodic() {
    boolean ballReady4Indexer = ballReadyForIndexer();
    boolean ballExiting = ballExiting();
    boolean ballStaged = ballStaged();

    //SmartDashboard.putNumber("Belt Amp", indexerBelts.getcurrent());

    SmartDashboard.putNumber("ball count", ballCount);
    SmartDashboard.putString("indexer state", mode.name());
    SmartDashboard.putNumber("Belt RPM", indexBelts.getSelectedSensorVelocity() * 600 / 2048);
    SmartDashboard.putNumber("Kicker RPM", indexKicker.getSelectedSensorVelocity() * 600 / 2048);

    SmartDashboard.putNumber("Agitator RPM", m_agitator_encoder.getVelocity());
    SmartDashboard.putNumber("Agitator Current", m_hopperAgitator.getOutputCurrent());

    if (mode == Mode.STOP) {
      stopIndexer();
    }
    if(mode == Mode.EJECT){
        setKickerPercentOutput(0.9);
        setBeltsPercentOutput(0.9);
        setHopperPercentOutput(0.5);
    }
    else if (mode == Mode.REVERSE) {
        setKickerPercentOutput(-0.5);
        setBeltsPercentOutput(-0.6);
        setHopperPercentOutput(-0.9);
    }
    else if (mode == Mode.INTAKE) {
      // Normal, non-eject mode
      SmartDashboard.putNumber("Eject State", 0);

      if (ballExiting) {
        // ball exiting, but we're not shooting so stop the belts and kicker
        stopKicker();
        stopBelts();
        if (ballReady4Indexer) {
          stopHopper();
        }
        else {
          // secondary intake is empty, so keep running the hopper
          setHopperPercentOutput(0.7);
        }
      }
      else if (ballReady4Indexer == false) {
        // no ball exiting
        // no ball staged
        // no ball in secondary intake, run hopper
        setHopperPercentOutput(0.7);
        setAgitatorRPM(Constants.indexConstants.agitatorRPM);
        setBeltsPercentOutput(0.0);
        setKickerPercentOutput(0.0);
      }
      else {
        // no ball exiting
        // no ball staged
        // ball in secondary intake, pull it into staged
        setBeltsPercentOutput(0.6);
        setIntakePercentOutput(0.6);
      }
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

  public void setHopperPercentOutput(double percent){
    setAgitatorRPM(Constants.indexConstants.agitatorRPM);
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

  public void setAgitatorRPM(double rpm){
    agitatorController.setIAccum(0.0);
    agitatorController.setReference(rpm, ControlType.kVelocity);
  }

  public void ejectOneBall() {

    ejectIndexer();

    if (mode == Mode.EJECT) {
      // we're already in ejectMode
      return;
    }

    /**
     * steps:
     * 1. run indexer until ball exiting (get ready to shoot)
     * 2. run indexer until ball not exiting (shooting!)
     * 3. stop indexer when ball ready to exit (ready for next shot)
     */
    mode = Mode.EJECT;
    ejectBallStep1 = true;
    ejectBallStep2 = ballExiting();
    ejectBallStep3 = false;

  }

  /**
   * enable Intake mode, pull balls into intake
   */
  public void setStopMode(){
    mode = Mode.STOP;
  }

  /**
   * enable Intake mode, pull balls into intake
   */
  public void setIntakeMode(){
    mode = Mode.INTAKE;
  }

  /**
   * enable Eject mode - eject balls for firing
   */
  public void setEjectMode(){
    mode = Mode.INTAKE;
  }

  /**
   * enable Reverse mode - reverse balls out of indexo through hopper
   */
  public void setReverseMode(){
    mode = Mode.REVERSE;
  }

  /** 
   * Stop all motors
   */
  public void stopIndexer() {
    setBeltsPercentOutput(0.0);
    setKickerPercentOutput(0.0);
    setIntakePercentOutput(0.0);
    setAgitatorRPM(0.0);
    // m_blinkin.solid_orange();
  }

  /**
   * ballReadyForIndexer - monitor sensor 1 for a ball ready to be indexed
   * 
   * @return true if a ball is waiting to be indexed
   */
  public boolean ballReadyForIndexer() {
    return ! Sensor1.get();
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
      setHopperPercentOutput(1);
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
      setIntakePercentOutput(-0.3);
      setBeltsPercentOutput(-0.3);
      setKickerPercentOutput(-0.4);
      // m_blinkin.strobe_red();
  }

  /**
   * ejectIndexer() - run all indexer motors at eject/shooting speeds
   */
  public void ejectIndexer() {
      setBeltsPercentOutput(0.8);
      setKickerPercentOutput(0.8);
      setHopperPercentOutput(0.3);
      setAgitatorRPM(0);
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
    setAgitatorRPM(0);
  }
  
  /**
   * runOnlyIntake() - run intake motor and stop the belts and kicker
   */
  public void runOnlyIntake() {
      setIntakePercentOutput(1);
      setBeltsRPM(0);
      setKickerPercentOutput(0);
      setAgitatorRPM(Constants.indexConstants.agitatorRPM);
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

}
