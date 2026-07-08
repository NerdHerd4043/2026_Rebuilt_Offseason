package frc.robot.subsystems.Intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

public class Intake extends SubsystemBase {
    private SparkMax intakingMotor = new SparkMax(IntakeConstants.intakingMotorID, MotorType.kBrushless);
    private SparkFlex articulatingMotor = new SparkFlex(IntakeConstants.articulatingMotorID, MotorType.kBrushless);

    private AbsoluteEncoder absEncoder;

    private PIDController pidController = new PIDController(IntakeConstants.p, IntakeConstants.i, IntakeConstants.d);

    private boolean resting = true;

    public Intake() {
        final SparkMaxConfig intakingMotorConfig = new SparkMaxConfig();
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

    @Override
    public void periodic() {

        if (!resting || !pidController.atSetpoint()) {
            articulatingMotor.setVoltage(pidController.calculate(absEncoder.getPosition() * 360));
        }

        super.periodic();
    }
}
