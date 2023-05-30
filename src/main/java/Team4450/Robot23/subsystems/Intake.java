package Team4450.Robot23.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import Team4450.Lib.FXEncoder;
import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static Team4450.Robot23.Constants.*;

public class Intake extends SubsystemBase
{
    private WPI_TalonFX     motor = new WPI_TalonFX(INTAKE_MOTOR);
    private FXEncoder       encoder = new FXEncoder(motor);

    private final double    MAX_POWER = .50;
    private double          maxCurrent, lastTimeCalled;
    private boolean         holdPosition;

    private SynchronousPID  controller = new SynchronousPID(getName() + "Hold", 0.2, 0, 0);

    public Intake()
    {
        Util.consoleLog();

        addChild("TalonFX", motor);
        addChild("Encoder", encoder);
        addChild(controller.getName(), controller);

        //motor.setInverted(true);

        //encoder.setInverted(true);
    }

    @Override
    public void periodic()
    {
        double current = motor.getStatorCurrent();

        if (current > maxCurrent) maxCurrent = current;
        
        // Periodic function called on each scheduler loop so we can use
        // it to run the pid controller to hold position.

        if (holdPosition)
        {
            double time = Util.getElaspedTime(lastTimeCalled);

            lastTimeCalled = Util.timeStamp();
    
            double power = controller.calculate(getPosition(), time);
    
            motor.set(power);
        }
    }
    
    /**
     * Sets motor power.
     * @param power + means intake cube, - means intake cone.
     */
    public void setPower(double power)
    {
        // If holding position, ignore zero power, non zero power turns off hold.

        if (holdPosition && power != 0) toggleHoldPosition();

        if (holdPosition) return;

        power = Util.clampValue(power, MAX_POWER);
        
        motor.set(power);
   }

    public void stop()
    {
        motor.stopMotor();
    }

    /**
     * Return encoder tick count.
     * @return The current tick count.
     */
    public int getPosition()
    {
        return encoder.get();
    }

    /**
     * Reset encoder to zero.
     */
    public void resetPosition()
    {
        encoder.reset();
    }

    /**
     * Return the motor's current draw.
     * @return Current draw in amps.
     */
    public double getMotorCurrent()
    {
        return motor.getStatorCurrent();
    }

    /**
     * Returns the max motor current seen since last reset.
     * @return Max current in amps.
     */
    public double getMaxMotorCurrent()
    {
        return maxCurrent;
    }

    /**
     * Reset max current tracking.
     */
    public void resetMaxCurrent()
    {
        maxCurrent = 0;
    }

    /**
     * Starts or stop winch position hold function. Non zero input
     * power also turns off hold function.
     */
    public void toggleHoldPosition()
    {
        if (!holdPosition) 
        {
            holdPosition = true;
            
            controller.reset();

            controller.setSetpoint(getPosition());
    
            lastTimeCalled = Util.timeStamp();
        }
        else
            holdPosition = false;

        Util.consoleLog("%b", holdPosition);

        updateDS();
    }

    public void updateDS()
    {
        SmartDashboard.putBoolean("HoldIntake", holdPosition);
    }
        
    @Override
	public void initSendable( SendableBuilder builder )
	{
        super.initSendable(builder);

		builder.addDoubleProperty("Motor Current", this::getMotorCurrent, null);
		builder.addDoubleProperty("Max Motor Current", this::getMaxMotorCurrent, null);
	}   	
}
