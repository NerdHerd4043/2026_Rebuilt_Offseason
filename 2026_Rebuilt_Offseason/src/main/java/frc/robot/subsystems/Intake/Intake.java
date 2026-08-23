package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;

public class Intake extends SubsystemBase {
    private SparkFlex intakingMotor = new SparkFlex(IntakeConstants.intakingMotorID, MotorType.kBrushless);
    private SparkFlex articulatingMotor = new SparkFlex(IntakeConstants.articulatingMotorID, MotorType.kBrushless);

    private CANcoder encoder = new CANcoder(28);

    private ProfiledPIDController pidController = new ProfiledPIDController(IntakeConstants.P, IntakeConstants.I,
            IntakeConstants.D, new TrapezoidProfile.Constraints(2, 5));

    private boolean resting = true;

    public Intake() {
        final SparkFlexConfig intakingMotorConfig = new SparkFlexConfig();
        final SparkFlexConfig articulatingMotorConfig = new SparkFlexConfig();

        intakingMotorConfig.idleMode(IdleMode.kBrake);
        articulatingMotorConfig.idleMode(IdleMode.kBrake);

        intakingMotor.configure(intakingMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        articulatingMotor.configure(articulatingMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        pidController.disableContinuousInput();
    }

    public Command runIntake() {
        return this.run(() -> {
            intakingMotor.set(IntakeConstants.intakeSpeed);
        }).finallyDo(() -> {
            intakingMotor.stopMotor();
        });
    }

    public void intakeToStartAngle() {
        resting = true;
        pidController.setGoal(IntakeConstants.startingAngle);
    }

    public void intakeToIntakeAngle() {
        resting = false;
        pidController.setGoal(IntakeConstants.intakeAngle);
    }

    public Command helpFeedBalls() {
        resting = false;

        Command moveIntakeUp = runOnce(() -> {
            pidController.setGoal(45);
        }).withTimeout(2);

        Command moveIntakeDown = runOnce(() -> {
            pidController.setGoal(0);
        }).withTimeout(2);

        Command helpFeedBallsCommand = Commands.sequence(moveIntakeUp, moveIntakeDown);

        return helpFeedBallsCommand;
    }

    @Override
    public void periodic() {

        if (!resting || !pidController.atSetpoint()) {
            articulatingMotor.setVoltage(pidController.calculate(encoder.getAbsolutePosition().getValueAsDouble()
                    * 360));
        }

    }
}
