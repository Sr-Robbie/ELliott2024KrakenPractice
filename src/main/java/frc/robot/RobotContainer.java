// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FeedShootCommand;
import frc.robot.commands.IntakeFeedCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.States.intakingState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Feed.Feed;
//import frc.robot.subsystems.Feed.States.feedState;
import frc.robot.subsystems.Feed.States.feedState;


public class RobotContainer {
  private ShuffleboardTab autoTab;
  private SendableChooser<Command> autoChooser;
 // private Field2d field;



  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private Intake intake;
  private Feed feed;
  private Shooter shooter;
  //private IntakeFeedCommand intakeFeedCommand; 
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

 

  public RobotContainer() {
    createSubsystems();
    configureBindings();
    registerNamedCommands();
    setupTabs();
  }

  public void createSubsystems(){
    intake = new Intake();
    feed = new Feed();
    shooter = new Shooter();
  }

  private void setupTabs() {
    autoChooser = new SendableChooser<>();

    autoTab = Shuffleboard.getTab("Auto");
    autoTab.add(autoChooser).withSize(2, 1);

    autoChooser.setDefaultOption("Intake Auto", new PathPlannerAuto("Intake"));
    autoChooser.addOption("Circle Auto", new PathPlannerAuto("Circle"));
    autoChooser.addOption("Shooter Auto", new PathPlannerAuto("Shooter auto"));
  }


  public void configureBindings() {

    joystick.leftTrigger(0.1).whileTrue(new IntakeFeedCommand(intake, feed, .9, 0.9));
    joystick.rightTrigger(0.1).whileTrue(new IntakeFeedCommand(intake, feed, -0.9, -0.9));
    joystick.rightBumper().whileTrue(new FeedShootCommand(shooter, feed ,.8 ,0.5));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().onTrue(drivetrain.reset());
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(270)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void registerNamedCommands () {
    NamedCommands.registerCommand("Intake", new IntakeFeedCommand(intake, feed, .9, .9).withTimeout(1.0));
    NamedCommands.registerCommand("Shoot!", new FeedShootCommand(shooter, feed, .8, .5));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
