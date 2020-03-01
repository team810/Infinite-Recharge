/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CamMode;
import frc.robot.commands.Drive;
import frc.robot.commands.RunArmMotor;
import frc.robot.commands.RunCP;
import frc.robot.commands.RunFeed;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ToggleDriveMode;
import frc.robot.commands.ToggleSolenoid;
import frc.robot.commands.VisionMode;
import frc.robot.commands.Autonomous.CloseShooter;
import frc.robot.commands.Autonomous.DriveBack;
import frc.robot.commands.Autonomous.FarShooter;
import frc.robot.commands.Autonomous.SimpleAuto;
import frc.robot.commands.Autonomous.tuning;
//import frc.robot.commands.Autonomous.TurnToTarget;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  public DriveTrain m_drive = new DriveTrain();
  Shooter m_shoot = new Shooter();
  ControlPanel m_cp = new ControlPanel();
  Intake m_intake = new Intake();
  Climber m_climb = new Climber();
  Limelight m_limelight = new Limelight();

  private final Command tuning = new tuning(m_drive, 20);

  private final Joystick gamepad = new Joystick(5);
  public final Joystick gp = new Joystick(5);
  public final Joystick left = new Joystick(Constants.LEFT_STICK);
  public final Joystick right = new Joystick(Constants.RIGHT_STICK);

 // private final Command m_autoCommand = new TurnToTarget(m_drive, m_drive.set);//Robot.getAngleToTarget()
  //private final Command m_autoCommand = getAutonomousCommand();
  private final SimpleAuto m_simpleAuto = new SimpleAuto(m_drive, m_shoot, m_intake);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new Drive(m_limelight ,m_drive, 
                                                ()-> left.getRawAxis(1), () -> right.getRawAxis(1)));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
/** */
    final JoystickButton intake = new JoystickButton(gamepad, Constants.INTAKE_BTN);
    intake.whileHeld(new RunIntake(m_intake, -.6));

    final JoystickButton intakePrimary = new JoystickButton(right, Constants.INTAKE_BTN);
    intakePrimary.whileHeld(new RunIntake(m_intake, -.6));

    final JoystickButton feeder = new JoystickButton(gamepad, Constants.FEEDER_BTN);
    feeder.whileHeld(new RunFeed(m_intake, .4));

    final JoystickButton feederReverse = new JoystickButton(gamepad, Constants.FEED_REVERSE_BTN);
    feederReverse.whileHeld(new RunFeed(m_intake, -.4));

    //final JoystickButton farShoot = new JoystickButton(gamepad, Constants.FAR_SHOOT_BTN);
    //farShoot.whileHeld(new FarShooter(m_shoot, m_intake, m_drive));
    final JoystickButton farShoot = new JoystickButton(gamepad, Constants.FAR_SHOOT_BTN);
    farShoot.whileHeld(new FarShooter(m_shoot, m_intake, m_drive));

    final JoystickButton intakeSOL = new JoystickButton(gamepad, Constants.INTAKE_SOL_BTN);
    intakeSOL.whenPressed(new ToggleSolenoid(m_intake.intakeSOL));

    final JoystickButton cpSOL = new JoystickButton(gamepad, 2);
    cpSOL.whenPressed(new ToggleSolenoid(m_cp.control_solenoid));

    final JoystickButton closeShoot = new JoystickButton(gamepad, Constants.CLOSE_SHOOT_BTN);
    closeShoot.whileHeld(new CloseShooter(m_shoot, m_intake, m_drive));

    final JoystickButton armToggle = new JoystickButton(gamepad, Constants.ARMSOL_BTN);
    armToggle.whenPressed(new ToggleSolenoid(m_climb.switch_climb));

    final JoystickButton switchDriveMode = new JoystickButton(gamepad, Constants.DRIVE_MODE_BTN);
    switchDriveMode.whenPressed(new ToggleDriveMode(m_drive));

    final JoystickButton switchDriveMode2 = new JoystickButton(left, 2);
    switchDriveMode2.whenPressed(new ToggleDriveMode(m_drive));

    final JoystickButton switchShooter = new JoystickButton(gamepad, Constants.SWITCH_SHOOT);
    switchShooter.whenPressed(new ToggleSolenoid(m_shoot.shooterSOL));

    final JoystickButton visionMode = new JoystickButton(right, 3);
    visionMode.toggleWhenActive(new VisionMode(m_limelight));
    final JoystickButton camMode = new JoystickButton(right, 4);
    camMode.toggleWhenActive(new CamMode(m_limelight));

    final JoystickButton runCP = new JoystickButton(gamepad, 11);
    runCP.whileHeld(new RunCP(m_cp));
  
    
    final JoystickButton driveBack = new JoystickButton(left, 2);
    driveBack.whileHeld(new DriveBack(m_drive));

    //final JoystickButton winch = new JoystickButton(gamepad, 9);
    //winch.whileHeld(new RunWinch(m_climb));

    final JoystickButton runArm = new JoystickButton(gamepad, 7);
    runArm.whileHeld(new RunArmMotor(m_climb, .3));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.ksVolts,
      Constants.kvVoltSecondsPerMeter,
      Constants.kaVoltSecondsSquaredPerMeter),
        m_drive.getKinematics(),
        10);

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(11), Units.feetToMeters(2));
    config.setKinematics(m_drive.getKinematics());
    config.addConstraint(autoVoltageConstraint);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                              new Pose2d(0, 0, new Rotation2d(0)),
                              List.of(
                                new Translation2d(0.5, 0.5),
                                new Translation2d(1, 1)
                              ), 
                              new Pose2d(2, 2, new Rotation2d(0)),
                              config);
    RamseteCommand command = new RamseteCommand(
    trajectory, 
    m_drive::getPose, 
    new RamseteController(2.,.7),
    m_drive.getFeedForward(),
    m_drive.getKinematics(),
    m_drive::getSpeeds,
    m_drive.getLeftPIDController(), 
    m_drive.getRightPIDController(),
    m_drive::tankDriveVolts, 
    m_drive);

    return command.andThen(() -> m_drive.tankDriveVolts(0, 0)); 
    //return m_simpleAuto;
  }
}
