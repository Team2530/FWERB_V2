package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;


public class ElevatorSubsystem extends ProfiledPIDSubsystem  {
    // NOTE: Elevator motor one has both the encoder used for positioning, and the limit switch used for zeroing
    private final CANSparkFlex elevatorMotorOne = new CANSparkFlex(Constants.Elevator.elevatorOnePort, MotorType.kBrushless);
    private final CANSparkFlex elevatorMotorTwo = new CANSparkFlex(Constants.Elevator.elevatorOnePort, MotorType.kBrushless);
    private final ElevatorFeedforward feedForward = new ElevatorFeedforward(
        Constants.Elevator.Feedforward.Ks,        
        Constants.Elevator.Feedforward.Kg,
        Constants.Elevator.Feedforward.Kv,
        Constants.Elevator.Feedforward.Ka

    );

    private boolean isZeroed = false;
    private final RelativeEncoder elevatorEncoder = elevatorMotorOne.getEncoder();
    private final SparkLimitSwitch bottomLimit;        

    public ElevatorSubsystem() {
        super(
            new ProfiledPIDController(
                Constants.Elevator.PID.kP, 
                Constants.Elevator.PID.kI, 
                Constants.Elevator.PID.kD,
                new TrapezoidProfile.Constraints(
                    Constants.Elevator.PID.MAX_VELOCITY,
                    Constants.Elevator.PID.MAX_ACCELERATION
                )
            ),
        0.0
        );
        
        // TODO: Check that this is an acceptable tolerance!
        this.getController().setTolerance(Units.inchesToMeters(0.5));

        isZeroed = false;

        elevatorMotorOne.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        elevatorMotorTwo.setIdleMode(CANSparkFlex.IdleMode.kBrake);

        elevatorEncoder.setPositionConversionFactor(1.0/Constants.Elevator.motorTurnsPerMeter);
        elevatorEncoder.setVelocityConversionFactor(1.0/Constants.Elevator.motorTurnsPerMeter);
        elevatorEncoder.setInverted(Constants.Elevator.elevatorEncoderInverted);

        bottomLimit = elevatorMotorOne.getReverseLimitSwitch(Constants.Elevator.bottomLimitMode);

        // todo check if offset encoders
        // todo check if motors inverted
        // todo set elevator tolerance
    }    

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // TODO: Elevator feedforward
        // double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        double ff = feedForward.calculate(setpoint.position, setpoint.velocity);

        if (isZeroed) {
            elevatorMotorOne.setVoltage(output + ff);
            elevatorMotorTwo.setVoltage(output + ff);
        } else {
            // Move down a little bit to zero
            elevatorMotorOne.set(-0.05);
            elevatorMotorTwo.set(-0.05);
        }
    }

    @Override
    public double getMeasurement() {
        return elevatorEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();

        if (bottomLimit.isPressed() && (!isZeroed)) {
            elevatorEncoder.setPosition(0.0);
            setGoal(0.0);
            isZeroed = true;
        }
    }

    public boolean isInPosition() {
        return this.getController().atGoal();
    }
}