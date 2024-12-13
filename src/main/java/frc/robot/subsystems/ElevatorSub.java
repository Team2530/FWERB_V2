package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;


public class ElevatorSub extends SubsystemBase {
    private boolean enablePID = false;

    private final CANSparkMax elevatorMotorOne = new CANSparkMax(Constants.Elevator.elevatorOnePort, MotorType.kBrushless);
    private final CANSparkMax elevatorMotorTwo = new CANSparkMax(Constants.Elevator.elevatorOnePort, MotorType.kBrushless);

    private final AbsoluteEncoder elevatorEncoder = elevatorMotorOne.getAbsoluteEncoder(Type.kDutyCycle);

    ProfiledPIDController elevatorPIDController =
        new ProfiledPIDController(Constants.Elevator.PID.kP, Constants.Elevator.PID.kI, Constants.Elevator.PID.kD,
            new TrapezoidProfile.Constraints(Constants.Elevator.PID.MAX_VELOCITY,
                Constants.Elevator.PID.MAX_ACCELERATION));

    public ElevatorSub(int motorPort) {
        elevatorMotorOne.restoreFactoryDefaults();
        elevatorMotorTwo.restoreFactoryDefaults();
        elevatorMotorOne.setIdleMode(CANSparkMax.IdleMode.kBrake);
        elevatorMotorTwo.setIdleMode(CANSparkMax.IdleMode.kBrake);
        elevatorEncoder.setPositionConversionFactor(360);
        elevatorEncoder.setVelocityConversionFactor(360);
        elevatorEncoder.setInverted(true);
        // todo check if offset encoders
        // todo check if motors inverted
        // todo set elevator tolerance
        //todo enable PID

    }
    /**
     * Get angle of elevator in degrees with a crossover outside of the configuration space
     */
    public double getElevatorAngle() {
        double angle = elevatorEncoder.getPosition();
        if (angle > Constants.Elevator.PID.TURNOVER_THRESHOLD) {
            angle -= 360;
        }
        return angle;
    }

    /**
     * Get angle of elevator in radians with a crossover outside of the configuration space
     */
    public double getelevatorAngleRad() {
        return getElevatorAngle() * Math.PI / 180.0;
    }


    /**
     * Set setpoint for elevator angle in degrees
     */
    public void setelevatorGoal(double goal) {
        enablePID = true;
        if (goal > Constants.Elevator.PID.TURNOVER_THRESHOLD) {
            goal -= 360;
        }
        elevatorPIDController.setGoal(goal * Math.PI / 180.0);
    }
    

    /**
     * Get if elevator angle is close to the setpoint
     */
    public boolean elevatorInPosition() {
        return elevatorPIDController.atGoal();
    }


    /**
     * Set elevator to extended position
     */
    public void extendelevator() {
    }

    /**
     * Set elevator to retracted position
     */
    public void retractelevator() {
    }

    /**
     * Set motors to coast mode
     */
    public void setCoastMode() {
        elevatorMotorOne.setIdleMode(IdleMode.kCoast);
        elevatorMotorTwo.setIdleMode(IdleMode.kCoast);
    }

    /**
     * Set motors to brake mode
     */
    public void setBrakeMode() {
        elevatorMotorOne.setIdleMode(IdleMode.kBrake);
        elevatorMotorTwo.setIdleMode(IdleMode.kBrake);
    }

    public void periodic() {
        if (enablePID) {
            double output = elevatorPIDController.calculate(getElevatorAngle());
            elevatorMotorOne.set(output);
            elevatorMotorTwo.set(output);
        }
    }

    public void enablePID() {
        enablePID = true;
    }

    public void disablePID() {
        enablePID = false;
    }

    public void setElevatorPosition(double position) {
        enablePID();
        setelevatorGoal(position);
    }
}