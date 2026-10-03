// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.RollerFloor.RollerFloor;
import frc.robot.subsystems.Shooter.FlyWheel;
import frc.robot.subsystems.Shooter.Indexer;
import frc.robot.subsystems.drivebase.DriveConstants;
import frc.robot.subsystems.drivebase.Drivebase;
import cowlib.Util;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController controller = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  FlyWheel flyWheel = new FlyWheel();
  RollerFloor rollerFloor = new RollerFloor();
  Indexer indexer = new Indexer();
  Drivebase drivebase = new Drivebase();
  Intake intake = new Intake();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            this::getScaledXY,
            () -> scaleRotationAxis(controller.getRightX())));

    intake.setDefaultCommand(
        intake.adjustSetPoint(() -> controller.getLeftTriggerAxis() - controller.getRightTriggerAxis()));

    CommandScheduler.getInstance().schedule(flyWheel.stopFlyWheel());

    // Configure the trigger bindings
    configureBindings();
  }

  // Used to create an area around the center of the joystick where the input is
  // 0, so as to avoid stick drift.
  private double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    } else {
      return input;
    }
  }

  // Changes our input -> output from linear to exponential, allowing finer
  // control close to the center without limiting our max output (since 1 is the
  // highest input and 1^2 (the ouput) = 1, so no change to the edges of the
  // input/output)
  private double[] getScaledXY() {
    // Array for storing the x/y inputs from the controller
    double[] xy = new double[2];

    // Assigning inputs to array locations. X and Y are switched because the
    // controller is funky.
    xy[0] = deadband(-controller.getLeftY(), DriveConstants.deadband);
    xy[1] = deadband(-controller.getLeftX(), DriveConstants.deadband);

    Util.square2DVector(xy);

    return xy;
  }

  private double scaleRotationAxis(double input) {
    return this.deadband(this.squared(input), DriveConstants.deadband) *
        drivebase.getMaxAngleVelocity() * -0.6;
  }

  private double squared(double input) {
    return Math.copySign(input * input, input);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // flywheel PID
    controller.rightBumper().onTrue(flyWheel.runFlyWheel());
    controller.rightBumper().onFalse(flyWheel.stopFlyWheel());

    // roller floor and shooter feeding
    controller.b().whileTrue(rollerFloor.feedCommand());
    controller.a().whileTrue(Commands.parallel(indexer.indexAndKickupCommand(), rollerFloor.feedCommand()));

    // intake PID
    controller.x().whileTrue(intake.helpFeedBalls());
    controller.povDown().onTrue(intake.actavtivePID());
    controller.povLeft().onTrue(intake.intakeToIntakeAngle());
    controller.povRight().onTrue(intake.intakeToStartAngle());

    // intaking
    controller.leftBumper().whileTrue(intake.runIntake());
    controller.y().whileTrue(intake.runIntakeReverse());

    // gyro reset
    controller.povUp().onTrue(drivebase.resetGyro());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public void disabledInit() {
    intake.disabledInit();
  }
}
