package frc.robot.subsystems.Intake;

import edu.wpi.first.epilogue.Logged;

@Logged
public class IntakeConstants {
    public static final int intakingMotorID = 26;
    public static final int articulatingMotorID = 27;
    public static final int encoderID = 28;

    public static final double intakeSpeed = 1.0;

    public static final int articulatingMotorCurrent = 40;

    public static final double startingAngle = 2.61;
    public static final double intakeAngle = 0.0;

    public static final double testSetPoint = 1.05;

    public static final Double P = 8.0;

    public static final Double I = 0.0;
    public static final Double D = 0.0;

    public static final Double kS = 0.0;
    public static final Double kG = 0.5;
    public static final Double kV = 0.0;
}
