
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import javax.swing.JPopupMenu.Separator;

//import java.sql.DriverAction;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "go straight ";
  private static final String kCustomAuto = "shotspeaker";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private DifferentialDrive Helios;

  //drive----------------
  CANSparkMax M_L1;
  CANSparkMax M_R1;
  CANSparkMax M_L2;
  CANSparkMax M_R2;
  //----------------------

  //rollers-----------------
  CANSparkMax M_FLNCH;
  CANSparkMax M_BLNCH;
  int offset = 0;
  int SUCk =190;
  int low = 250;
  int psh = 200;
  //------------------------

  // scooper postitions------------------------------
  CANSparkMax M_ARM;
  CANSparkMax M_ARM2;
  int kArmPosConversion = 5;
  int kArmVelConversion = kArmPosConversion / 60;
  int kArmOffset = 100; // degrees
  int STW = 0; // degrees
  int SCOOP = 100; // degrees
  int AMP = 310;
  int Speeker = 100; // degrees
  int PULL = 310; // degrees
  int kArmMaxAngle = 360; // degrees
  int kArmMinAngle = 20; // degrees
  // -------------------------------------------

  int L1Id = 1;
  int R1Id = 2;
  int L2Id = 3;
  int R2Id = 4;
  int ARMId = 5;
  int ARM2Id =6;
  int FRLLId = 7;
  int BRLLId = 8;

  int CurrentLimit = 50;

  //PID------------------------

  AbsoluteEncoder m_armEncoder;
  AbsoluteEncoder backroll;
  AbsoluteEncoder frontroll;
  
  SparkPIDController m_armPIDController;
  SparkPIDController Frontrollpid;
  SparkPIDController Backrollpid;

  double Position = 0.05;
  double Iposition = 0.05;
  double Dposition = 0.005;
  double FF = 0.05;
  //---------------------------
  double startTime;
  double periodicTime;


  private XboxController driveController = new XboxController(1);
  private XboxController opController = new XboxController(0);

  private final DigitalInput SWTCH = new DigitalInput(0);

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    M_L1 = new CANSparkMax(L1Id, MotorType.kBrushless);
    M_R1 = new CANSparkMax(R1Id, MotorType.kBrushless);
    M_L2 = new CANSparkMax(L2Id, MotorType.kBrushless);
    M_R2 = new CANSparkMax(R2Id, MotorType.kBrushless);
    M_ARM = new CANSparkMax(ARMId, MotorType.kBrushed);
    M_FLNCH = new CANSparkMax(FRLLId, MotorType.kBrushless);
    M_BLNCH = new CANSparkMax(BRLLId, MotorType.kBrushless);


    M_L1.restoreFactoryDefaults();
    M_L2.restoreFactoryDefaults();
    M_R1.restoreFactoryDefaults();
    M_R2.restoreFactoryDefaults();
    M_ARM.restoreFactoryDefaults();
    M_ARM2.restoreFactoryDefaults();
    M_FLNCH.restoreFactoryDefaults();
    M_BLNCH.restoreFactoryDefaults();

    M_L2.follow(M_L1);
    M_R2.follow(M_R1);
    M_ARM2.follow(M_ARM);

    M_L1.setIdleMode(IdleMode.kCoast);
    M_R2.setIdleMode(IdleMode.kCoast);
    M_ARM.setIdleMode(IdleMode.kBrake);
    M_ARM2.setIdleMode(IdleMode.kBrake);
    M_FLNCH.setIdleMode(IdleMode.kBrake);
    M_BLNCH.setIdleMode(IdleMode.kBrake);

    M_L1.setSmartCurrentLimit(CurrentLimit);
    M_R1.setSmartCurrentLimit(CurrentLimit);
    M_L2.setSmartCurrentLimit(CurrentLimit);
    M_R2.setSmartCurrentLimit(CurrentLimit);
    M_ARM.setSmartCurrentLimit(CurrentLimit);
    M_ARM2.setSmartCurrentLimit(CurrentLimit);
    M_FLNCH.setSmartCurrentLimit(CurrentLimit);
    M_BLNCH.setSmartCurrentLimit(CurrentLimit);

    M_L1.setInverted(false);
    M_R1.setInverted(true);
    M_ARM.setInverted(false);
    M_ARM2.setInverted(true);
    M_FLNCH.setInverted(true);
    M_BLNCH.setInverted(true);
 
    //armpid------------------------------------------------------------
     m_armPIDController = M_ARM.getPIDController();
     m_armEncoder = M_ARM.getAbsoluteEncoder(Type.kDutyCycle);
     m_armPIDController.setFeedbackDevice(m_armEncoder);
     m_armEncoder.setPositionConversionFactor(360);
     m_armEncoder.setVelocityConversionFactor(4.0);
     m_armEncoder.setZeroOffset(kArmOffset);
    //--------------------------------------------------------------

    //backrollpid---------------------
     m_armPIDController = M_BLNCH.getPIDController();
     backroll = M_BLNCH.getAbsoluteEncoder(Type.kDutyCycle);
     m_armPIDController.setFeedbackDevice(frontroll);
     frontroll.setPositionConversionFactor(360);
     frontroll.setVelocityConversionFactor(5.0);
     frontroll.setZeroOffset(offset);
    //---------------------------------------------------

    //frontrollpid---------------------------------
     m_armPIDController = M_FLNCH.getPIDController();
     backroll = M_FLNCH.getAbsoluteEncoder(Type.kDutyCycle);
     m_armPIDController.setFeedbackDevice(backroll);
     backroll.setPositionConversionFactor(360);
     backroll.setVelocityConversionFactor(5.0);
     backroll.setZeroOffset(offset);
    //----------------------------------------------------



    m_armPIDController.setPositionPIDWrappingEnabled(false);
    m_armPIDController.setPositionPIDWrappingMinInput(kArmMinAngle);
    m_armPIDController.setPositionPIDWrappingMaxInput(kArmMaxAngle);

    m_armPIDController.setP(Position);
    m_armPIDController.setD(Dposition);

    /* burn flash to store settings */
    M_L1.burnFlash();
    M_L2.burnFlash();
    M_R1.burnFlash();
    M_R2.burnFlash();
    M_ARM.burnFlash();
    M_ARM2.burnFlash();
    M_FLNCH.burnFlash();
    M_BLNCH.burnFlash();

    Helios = new DifferentialDrive(M_L1, M_R1);


  }

  @Override
  public void robotPeriodic() {
  }

  /* This runs when robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
    m_autoSelected = m_chooser.getSelected();
    // m_autoselected = smartdashboard.getsring("Auto selector", kDefaultAuto);

  }

  @Override
  public void autonomousPeriodic() {
    if (m_autoSelected.equals(kDefaultAuto)) {
      if ((periodicTime - startTime > 0) && (periodicTime - startTime < 2)) {
        Helios.tankDrive(.4,.4);
      } else if ((periodicTime - startTime > 2) && (periodicTime - startTime < 10)) {
       Helios.tankDrive(.0,.5);
      }
      else if ((periodicTime - startTime < 7) && (periodicTime - startTime > 10)){
      m_armPIDController.setReference(AMP, Controltype.kPosition)
      }
      else if ((periodicTime - startTime < 12) && (periodicTime - startTime > 9)){
        
      }
    }

    else if (m_autoSelected.equals(kCustomAuto)) {
      if ((periodicTime - startTime > 0) && (periodicTime - startTime < 1)) {
        Helios.tankDrive(.4,.4);
      } else if ((periodicTime - startTime > 2) && (periodicTime - startTime < 5)) {
        Helios.tankDrive(.0,.6);
      }
      else if ((periodicTime - startTime < 7) && (periodicTime - startTime > 10)) {
        m_armPIDController.setReference(Speeker, Controltype.kPosition)
      }
      else if ((periodicTime - startTime < 9) && (periodicTime - startTime > 12)) {
        Backrollpid.setReference(SUCk, Controltype.kPosition);
      }
      else if (()) {

      }
    }
    periodicTime = Timer.getFPGATimestamp();

  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    Helios.tankDrive(-driveController.getRightY(), -driveController.getLeftY());
    
   //backrollers------------------------------------
      double opValue = opController.getRightY();
      double motorSpeed = opValue;
      M_BLNCH.set(motorSpeed);

      if (opController.getRightBumperPressed()) {
      Backrollpid.setReference(SUCk, ControlType.kPosition);
    }
      if (opController.getRightTriggerAxis() > .5) {
      Backrollpid.setReference(low, ControlType.kPosition);
    }
    
    //-------------------------------------------

    //front rollers----------------------------

      double opValue2 = opController.getLeftY();
      double motorSpeed2= opValue2;
      M_BLNCH.set(motorSpeed2);

      if (opController.getLeftBumperPressed()) {
      Frontrollpid.setReference(SUCk, ControlType.kPosition);
      }
      if (opController.getLeftTriggerAxis() > .5) {
      Backrollpid.setReference(psh, ControlType.kPosition);
    }
    //---------------------------------------------------

    /* arm */

    if (opController.getBButtonPressed()) {
      m_armPIDController.setReference(STW, ControlType.kPosition);
    }
    if (opController.getYButtonPressed()) {
      m_armPIDController.setReference(SCOOP, ControlType.kPosition);
    }
    if (opController.getXButtonPressed()) {
      m_armPIDController.setReference(AMP, ControlType.kPosition);
    }
    if (opController.getAButtonPressed()) {
      m_armPIDController.setReference(Speeker, ControlType.kPosition);
    }

    SmartDashboard.getBoolean("SWTCH", false);

    SWTCH.get();
    SmartDashboard.putBoolean("SWTCH", SWTCH.get());


    SmartDashboard.putNumber("arm Value", m_armEncoder.getPosition());
    SmartDashboard.putNumber("back Value", backroll.getPosition());
    SmartDashboard.putNumber("front Value", frontroll.getPosition());
  }
}

 