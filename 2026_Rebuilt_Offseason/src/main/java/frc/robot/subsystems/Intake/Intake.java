package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;

public class Intake extends SubsystemBase {
    private SparkFlex intakingMotor = new SparkFlex(IntakeConstants.intakingMotorID, MotorType.kBrushless);
    private SparkFlex articulatingMotor = new SparkFlex(IntakeConstants.articulatingMotorID, MotorType.kBrushless);

    private CANcoder encoder = new CANcoder(28);

    private PIDController pidController = new PIDController(IntakeConstants.p, IntakeConstants.i, IntakeConstants.d);

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
        pidController.setSetpoint(IntakeConstants.startingAngle);
    }

    public void intakeToIntakeAngle() {
        pidController.setSetpoint(IntakeConstants.intakeAngle);
    }

    public Command helpFeedBalls() {
        Command moveIntakeUp = runOnce(() -> {
            pidController.setSetpoint(45);
        }).withTimeout(2);

        Command moveIntakeDown = runOnce(() -> {
            pidController.setSetpoint(0);
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
