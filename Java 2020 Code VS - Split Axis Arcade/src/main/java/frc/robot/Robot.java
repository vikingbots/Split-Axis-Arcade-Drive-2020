/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  
  private final int XBOX_CONTROLLER_PORT = 0;
//This is all drive train

  private final int RightMotor1Port = 0;
  private final int RightMotor2Port = 1;
  private final int LeftMotor1Port = 2;
  private final int LeftMotor2Port = 3;
  private final int VerticalMotorPort = 4;
  private final double destinationAngleUp = 90;
  private final double destinationAngleDown = 0;
  private final double StartingAngleIntake = 1234890;
  //change this when you figure out the starting angle

  private final double Auton_Forward_Time_Limit = 3.0;
  private final double Auton_Motor_Forward_Power = 0.5;
  private final double DistancePerPulse = .0092;
  private final CounterBase.EncodingType EncoderType = EncodingType.k4X;

  private Boolean pressureSwitch;
  private final double creepModeMultiplier = 0.5;
  private double accelResult;

  private Talon LeftMotor1, RightMotor1, LeftMotor2, RightMotor2, VerticalMotor;
  private XboxController ArcadeController;
  private Solenoid solenoid1;

  private Compressor compressor1;

  private Timer Timer01;
  private Encoder Encoder01;
  private int leftchannelA;
  private int leftchannelB;
  private Boolean EncoderReverseDirection;

  private double yThrottle;
  private double xRotation;
  private boolean squareInputs = true;
  private double RightSpeed = 0;
  private double LeftSpeed = 0;

  private double left, right, vertical;

  private double GyroAngle;
  
  //Dashboard
  //SmartDashboard dash = new SmartDashboard();
  String AutoModeSelected;

  //Auton Variables
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kEncoderAuto = "Encoder Auto";
  private static String AutonModes[] = new String[]{kDefaultAuto, kCustomAuto, kEncoderAuto};
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private int auton_phase = 1;

  //Gyro setup
  private final ADXRS450_Gyro Z_AXIS_GYRO = new ADXRS450_Gyro();
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("Encoder Auto",kEncoderAuto);
    //dash.putStringArray("Auto List", AutonModes);

    LeftMotor1 = new Talon(LeftMotor1Port);
    LeftMotor2 = new Talon(LeftMotor2Port);
    
    
    //LeftSCGroup = new SpeedControllerGroup(LeftMotor1, LeftMotor2);
    //LeftSCGroup.setInverted (true);

    RightMotor1 = new Talon(RightMotor1Port);
    RightMotor2 = new Talon(RightMotor2Port);
    //RightSCGroup = new SpeedControllerGroup(RightMotor1, RightMotor2);

    //DifferentialDrive ArcadeDrive = new DifferentialDrive(LeftSCGroup, RightSCGroup);

    VerticalMotor = new Talon(VerticalMotorPort);

    LeftMotor1.setInverted (true);
    LeftMotor2.setInverted (true);

    solenoid1 = new Solenoid(2);

    compressor1 = new Compressor(0);

    leftchannelA = 0;
    leftchannelB = 1;
    EncoderReverseDirection = true;
    Encoder01 = new Encoder(leftchannelA, leftchannelB, EncoderReverseDirection, EncoderType);
    Encoder01.setDistancePerPulse(DistancePerPulse);

    //leftStick = new Joystick(LEFT_STICK_USB_PORT);
    //rightStick = new Joystick(RIGHT_STICK_USB_PORT);
    ArcadeController = new XboxController(XBOX_CONTROLLER_PORT);

    Timer01 = new Timer();


   //CameraServer.getInstance().startAutomaticCapture();
  }

  /**
   * @return
   * @return the encoder01
   
  public Encoder Encoder01() {
    return Encoder01;
  }*/

  /**
   * @param encoder01 the encoder01 to set
   
  public void setEncoder01(Encoder encoder01) {
    this.Encoder01 = encoder01;

  }*/

  /**
   * @return the leftMotor1Port
   
  public int getLeftMotor1Port() {
    return LeftMotor1Port;
  }*/

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  //Periodic Tasks
  {
    //ArcadeDrive.tankDrive(LeftSpeed, RightSpeed);

    LeftMotor1.set(LeftSpeed);
    LeftMotor2.set(LeftSpeed);
    RightMotor1.set(RightSpeed);
    RightMotor2.set(RightSpeed);

    //ArcadeDrive.arcadeDrive(0, 0, false);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    //m_autoSelected = m_chooser.getSelected();
     m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    Timer01.reset();
    Timer01.start();
  }

  
   // This function is called periodically during autonomous.
   
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        if (Timer01.get() < Auton_Forward_Time_Limit) 
        {
          LeftMotor1.set(Auton_Motor_Forward_Power);
          LeftMotor2.set(Auton_Motor_Forward_Power);
          RightMotor1.set(Auton_Motor_Forward_Power);
          RightMotor2.set(Auton_Motor_Forward_Power);
        }
        else 
        {
          RobotStop();
        }
        break;
      case kDefaultAuto:
        RobotStop();
        break;
      case kEncoderAuto:
      switch (auton_phase){
        case 1:
        AutonStraight(0.5, 6.0);
        case 2:
        //Auton Phase 2
      }
      default:
        RobotStop();
        break;
    }
  }
  // auto move function                       (seconds)
  /*public void automoveforwards(double speed, double time){
    
  }*/
  
  // an acceleration function, this is used to change a number by speedInterval (acceleration) to reach speedGoal each time it runs
  public double acceleratenum (double speedCurrent, double speedGoal, double speedInterval) {
    if (speedCurrent < speedGoal) {
      if (speedCurrent + speedInterval > speedGoal){
        accelResult = speedGoal;
      }
      else {accelResult = speedCurrent + speedInterval;}
    }
    else if (speedCurrent > speedGoal) {
      if (speedCurrent - speedInterval < speedGoal){
        accelResult = speedGoal;
      }
      else {accelResult = speedCurrent - speedInterval;}
    } 
    else {
    accelResult = speedCurrent;
    }
    return accelResult;
  } 
   // This function is called periodically during operator control.

  @Override
  public void teleopPeriodic() {
    //THIS IS TELEOP
    yThrottle = ArcadeController.getRawAxis(1);
    xRotation = ArcadeController.getRawAxis(4);

    RightSpeed = yThrottle + xRotation;
    LeftSpeed = yThrottle - xRotation;
    
    //This statement is so that when right trigger is pushed, it enters "Beast-mode" (Boost-Mode)
    //Default code is programmed below; "Default" or the else statement is written above (functioning at 100%) 
    if(!ArcadeController.getRawButton(6))
    {
        LeftSpeed = LeftSpeed * 0.65;
        RightSpeed = RightSpeed * 0.65;
    }

    //Gyro Angle Stuff
    GyroAngle = Z_AXIS_GYRO.getAngle();
    System.out.println(GyroAngle);

    /*if(rightStick.getTrigger())
    {
    left = leftStick.getY()*creepModeMultiplier;
    right = rightStick.getY()*creepModeMultiplier;
    } else 
    {
    left  = leftStick.getY();
    right = rightStick.getY();
    }
    LeftMotor1.set(left);
    LeftMotor2.set(left);
    RightMotor1.set(right);
    RightMotor2.set(right);

    if(leftStick.getTrigger())
    {
      solenoid1.set(true);
    }
    else
    {
      solenoid1.set(false);
    }
    
    switch(rightStick.getPOV()) {
      case 0: VerticalMotor.set(1); break;
      case 180: VerticalMotor.set(-1); break;
      default: VerticalMotor.set(0); break;
      */
    } 

    //console output
   // This function is called periodically during test mode.

  
  @Override
  public void testPeriodic() {

  }

  public void ArmIntake(){
    //rotate intake motor inward
    //intakearmdirect -1
  }

  public void ArmUp(){
    //rotate arm control up (cant overshoot or it may break)
    //intakearmdirect 0
  }

  public void ArmDown(){
    //rotate arm control down 
  }

  public void ArmEject(){
    //rotate intake motor outward
    //intakearmdirect 1
  }

  public void AutonStraight(Double Speed, Double Distance) {
    if (Encoder01.getDistance() < Distance) {
      LeftSpeed = Speed;
      RightSpeed = Speed;
    } else {
        RobotStop();
        auton_phase++;
        Encoder01.reset();
    }
    //While Encorder < Distance Drive Speed
    //STOP ALL MOTORS!
  }

  public void AutonTurn(Double Angle, Double Speed, Boolean TurnDirection) {
    //TurnDirection True is right
    //While Gyro < Angle Specified turn at Speed Speed in TurnDirection (1,-1)
    //STOP ALL MOTORS!
  }

  public void RobotStop(){
    LeftMotor1.set(0.0);
    LeftMotor2.set(0.0);
    RightMotor1.set(0.0);
    RightMotor2.set(0.0);
  }
  
}
