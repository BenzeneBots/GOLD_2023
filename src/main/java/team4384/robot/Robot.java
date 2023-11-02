// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team4384.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import team4384.robot.constants.CTREConfigs;
import team4384.robot.constants.RobotMap;
import team4384.robot.subsystems.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private CANSparkMax armBase1 = new CANSparkMax(49, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax armBase2 = new CANSparkMax(50, CANSparkMaxLowLevel.MotorType.kBrushless);

  private CANSparkMax IntakeTurner = new CANSparkMax(51, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax Intake = new CANSparkMax(52, CANSparkMaxLowLevel.MotorType.kBrushless);

  private Joystick armBase = new Joystick(1);
  private JoystickButton ArmBaseUp = new JoystickButton(armBase, 7);
  private JoystickButton ArmBaseDown = new JoystickButton(armBase, 8);
  private JoystickButton IntakeUp = new JoystickButton(armBase, 1);
  private JoystickButton IntakeDown = new JoystickButton(armBase, 2);
  private JoystickButton CubeOut = new JoystickButton(armBase, 3);
  private JoystickButton CubeIn = new JoystickButton(armBase, 4);
  private JoystickButton ConeIn = new JoystickButton(armBase, 5);
  private JoystickButton ConeOut = new JoystickButton(armBase, 6);

  private DigitalInput arm_limit = new DigitalInput(1);
  private DigitalInput bottom_intake = new DigitalInput(2);

  public final Swerve s_Swerve = new Swerve();


  private Command m_autonomousCommand;
  private boolean runOnce = false;


  SendableChooser<Integer> pathChosen;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    UsbCamera camera = CameraServer.startAutomaticCapture();
    CameraServer.putVideo("Video", 600, 400);
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    SendableChooser<Integer> pathChooser = new SendableChooser<>();
    pathChooser.setDefaultOption("Cube", 0);
    pathChooser.addOption("Cone", 1);
    pathChooser.addOption("Charge Station", 2);

    SmartDashboard.putData("Path Selection", pathChooser);
    armBase2.setInverted(true);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
k  * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Roll", m_robotContainer.s_Swerve.gyro.getRoll());
    m_robotContainer.UpdateSmartBoard();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    runOnce = false;
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    pathChosen = (SendableChooser<Integer>)SmartDashboard.getData("Path Selection");
    runOnce = false;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (runOnce) return;
    if (pathChosen.getSelected() == 0) {
      m_robotContainer.autonomous.basic();
    }
    else if (pathChosen.getSelected() == 1) {
      m_robotContainer.autonomous.cone();
    }
    else if (pathChosen.getSelected() == 2) {
      m_robotContainer.autonomous.chargingStation();
    }
    runOnce = true;

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putBoolean("Limit Switch", bottom_intake.get());

    if(ArmBaseUp.getAsBoolean()) {
      armBase1.set(RobotMap.BASE_SPEED);
      armBase2.set(RobotMap.BASE_SPEED);
    } else if(ArmBaseDown.getAsBoolean() && arm_limit.get() == false) {
      armBase1.set(-RobotMap.BASE_SPEED);
      armBase2.set(-RobotMap.BASE_SPEED);
    } else {
      armBase1.getPIDController().setReference(armBase1.getEncoder().getPosition(), CANSparkMax.ControlType.kPosition);
      armBase2.getPIDController().setReference(armBase2.getEncoder().getPosition(), CANSparkMax.ControlType.kPosition);
    }

    if(IntakeUp.getAsBoolean() && bottom_intake.get() == false) {
      IntakeTurner.set(RobotMap.INTAKE_TURNER_SPEED);
    } else if(IntakeDown.getAsBoolean()) {
      IntakeTurner.set(-RobotMap.INTAKE_TURNER_SPEED);
    } else {
      IntakeTurner.getPIDController().setReference(IntakeTurner.getEncoder().getPosition(), CANSparkMax.ControlType.kPosition);
      IntakeTurner.getPIDController().setReference(IntakeTurner.getEncoder().getPosition(), CANSparkMax.ControlType.kPosition);
    }

    if(CubeIn.getAsBoolean() || ConeOut.getAsBoolean()) {
      Intake.set(RobotMap.INTAKE_SPEED);
      SmartDashboard.putBoolean("Intake", true);
    } else if(CubeOut.getAsBoolean() || ConeIn.getAsBoolean()) {
      SmartDashboard.putBoolean("Intake", false);
      Intake.set(-RobotMap.INTAKE_SPEED);
    } else {
      Intake.stopMotor();
    }
  };

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  public static void sleep() {
    try {
      Thread.sleep(50);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    Intake.set(0.1);
    sleep();
    Intake.set(-0.1);
    sleep();
    IntakeTurner.set(0.1);
    sleep();
    IntakeTurner.set(-0.1);
    sleep();
    armBase1.set(0.1);
    armBase2.set(0.1);
    sleep();
    armBase1.set(-0.1);
    armBase2.set(-0.1);
    sleep();
    s_Swerve.drive(
            new Translation2d(0.01,0),
            0,
            false,
            false
    );
    sleep();
    s_Swerve.drive(
            new Translation2d(0,0.01),
            0,
            false,
            false
    );
    sleep();
    s_Swerve.drive(
            new Translation2d(0,0),
            0.1,
            false,
            false
    );
  }
}
