// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
    



public class Robot extends TimedRobot {

  
  private static final String kDefaultAuto = "out of coummunity";
  private static final String kCustomAuto = "charge station";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  CANSparkMax m_leftLeadDriveMotor;
  CANSparkMax m_rightLeadDriveMotor;
  CANSparkMax m_leftFollowDriveMotor;
  CANSparkMax m_rightFollowDriveMotor;

DoubleSolenoid m_clawSolenoid;
 
  int kClawForward = 0;
  int kClawReverse = 1;

  CANSparkMax m_ArmSwivelMotor;
 int kArmPosConversion = 5;
 int kArmVelConversion = kArmPosConversion/60;
 int kArmOffset = 100; //degrees
 int kArmStow = 32; //degrees
 int kArmHoriz = 260; //degrees
 int kArmGround = 310; //degrees
 int kArmSlope = 230; //degrees
 int kArmDub = 110; //degrees
 int kArmMaxAngle = 315; //degrees
 int kArmMinAngle = 30; //degrees

  double Position = 0.05;
  double Iposition = 0.05;
  double Dposition = 0.005;
  double FF = 0.05;

  private DifferentialDrive m_myRobot;

 private XboxController driveController = new XboxController(1);
 private XboxController opController = new XboxController(0);
  
  int kLeftLeadMotorID = 1;
  int kRightLeadMotorID = 2;
  int kLeftFollowMotorID = 3;
  int kRightFollowMotorID = 4;
  int kArmSwivelMotor = 5;

  int kMotorCurrentLimit = 50;

  AbsoluteEncoder m_armEncoder; 
  SparkMaxPIDController m_armPIDController;


  double startTime;
  double periodicTime;



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    m_leftLeadDriveMotor = new CANSparkMax(kLeftLeadMotorID, MotorType.kBrushless);
    m_rightLeadDriveMotor = new CANSparkMax(kRightLeadMotorID, MotorType.kBrushless);
    m_leftFollowDriveMotor = new CANSparkMax(kLeftFollowMotorID, MotorType.kBrushless);
    m_rightFollowDriveMotor = new CANSparkMax(kRightFollowMotorID, MotorType.kBrushless);
    m_ArmSwivelMotor = new CANSparkMax(kArmSwivelMotor, MotorType.kBrushed);

    m_leftFollowDriveMotor.restoreFactoryDefaults();
    m_leftLeadDriveMotor.restoreFactoryDefaults();
    m_rightFollowDriveMotor.restoreFactoryDefaults();
    m_rightLeadDriveMotor.restoreFactoryDefaults();
    m_ArmSwivelMotor.restoreFactoryDefaults();

    m_leftLeadDriveMotor.setIdleMode(IdleMode.kCoast);
    m_rightFollowDriveMotor.setIdleMode(IdleMode.kCoast);
    m_ArmSwivelMotor.setIdleMode(IdleMode.kBrake);

    m_leftLeadDriveMotor.setSmartCurrentLimit(kMotorCurrentLimit);
    m_rightLeadDriveMotor.setSmartCurrentLimit(kMotorCurrentLimit);
    m_leftFollowDriveMotor.setSmartCurrentLimit(kMotorCurrentLimit);
    m_rightFollowDriveMotor.setSmartCurrentLimit(kMotorCurrentLimit);
    m_ArmSwivelMotor.setSmartCurrentLimit(kMotorCurrentLimit);

    m_leftFollowDriveMotor.follow(m_leftLeadDriveMotor);
    m_rightFollowDriveMotor.follow(m_rightLeadDriveMotor);

    m_rightLeadDriveMotor.setInverted(false);
    m_leftLeadDriveMotor.setInverted(true);
    m_ArmSwivelMotor.setInverted(false);

    m_armPIDController = m_ArmSwivelMotor.getPIDController();
    m_armEncoder = m_ArmSwivelMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_armPIDController.setFeedbackDevice(m_armEncoder);

    m_armEncoder.setPositionConversionFactor(360);
    m_armEncoder.setVelocityConversionFactor(6.0);
    m_armEncoder.setZeroOffset(kArmOffset);

    m_armPIDController.setPositionPIDWrappingEnabled(false);
    m_armPIDController.setPositionPIDWrappingMinInput(kArmMinAngle);
    m_armPIDController.setPositionPIDWrappingMaxInput(kArmMaxAngle);


    m_armPIDController.setP(Position);
    //m_armPIDController.setI(Iposition);
    m_armPIDController.setD(Dposition);
    //m_armPIDController.setFF(FF);

/*claw configuration */
    m_clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kClawForward, kClawReverse);

/*burn flash to store settings */
    m_leftLeadDriveMotor.burnFlash();
    m_rightLeadDriveMotor.burnFlash();
    m_leftFollowDriveMotor.burnFlash();
    m_rightFollowDriveMotor.burnFlash();
    m_ArmSwivelMotor.burnFlash();

    m_myRobot = new DifferentialDrive(m_leftLeadDriveMotor, m_rightLeadDriveMotor);


    
  }
  
  @Override
  public void robotPeriodic() {}

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
    m_autoSelected = m_chooser.getSelected();
    // m_autoselected = smartdashboard.getsring("Auto selector", kDefaultAuto);
    
  }
  

  @Override
  public void autonomousPeriodic() {
    if (m_autoSelected.equals(kDefaultAuto)) {
      if((periodicTime-startTime>0) && (periodicTime-startTime<2)){
        m_armPIDController.setReference(kArmHoriz, ControlType.kPosition);
      }else if((periodicTime-startTime>2) && (periodicTime-startTime<8)){
        m_myRobot.tankDrive(.4, .4);
      }

    } 
    
    else if (m_autoSelected.equals(kCustomAuto)) {
      if((periodicTime-startTime>0) && (periodicTime-startTime<2)){
        m_armPIDController.setReference(kArmSlope, ControlType.kPosition);
        m_clawSolenoid.set(Value.kForward);
      }else if((periodicTime-startTime>2) && (periodicTime-startTime<5)){
        m_myRobot.tankDrive(.6, .6);
      }
    }
   periodicTime = Timer.getFPGATimestamp();
    
  }

  @Override
  public void teleopInit() {}


  @Override
  public void teleopPeriodic() {
      /*drive */
    m_myRobot.tankDrive(-opController.getRightY(), -opController.getLeftY());
    
  
    /* claw */
    if (opController.getBButtonPressed()) { 
      m_armPIDController.setReference(kArmStow, ControlType.kPosition); 
    }
    if (opController.getYButtonPressed()) { 
      m_armPIDController.setReference(kArmSlope, ControlType.kPosition); 
    }
    if (opController.getXButtonPressed()) { 
      m_armPIDController.setReference(kArmHoriz, ControlType.kPosition); 
    }
    if (opController.getAButtonPressed()) { 
      m_armPIDController.setReference(kArmGround, ControlType.kPosition); 
    }
    if (opController.getRightBumper()) { 
      m_armPIDController.setReference(kArmDub, ControlType.kPosition); 
    }

    /* arm */
    if (opController.getRightTriggerAxis() > .5 ) { m_clawSolenoid.set(Value.kForward); }
    if (opController.getLeftTriggerAxis() > .5 ) { m_clawSolenoid.set(Value.kReverse); }

    SmartDashboard.putNumber("Encoder Value", m_armEncoder.getPosition());
  }
}
