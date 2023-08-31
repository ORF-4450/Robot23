package Team4450.Robot23.commands;

import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.RobotContainer;
import Team4450.Robot23.subsystems.DriveBase;
import Team4450.Robot23.subsystems.PhotonVision;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command uses target information returned by Photovision API, which
 * returns data from PV running on the LimeLight. The targets yaw from the
 * camera and the area of the target in the field of vision are used by two
 * PID controllers to move the robot side to side and foward to scoring
 * position. A third controller uses the NavX yaw to correct any rotation
 * away from facing the scoring wall.
 */
public class AlignToTape extends CommandBase
{
    private PhotonVision            photonVision;
    private PhotonPipelineResult    result;
    private DriveBase               driveBase;
    private SynchronousPID          strafeController = new SynchronousPID("AlignToTape", 0.03, .003, .003);
    private SynchronousPID          throttleController = new SynchronousPID("DriveToTape", 0.20, .020, .020);
    private SynchronousPID          rotateController = new SynchronousPID("RotateToTape", 0.03, .003, .003);
    private final double            maxSpeed = .15;
    private double                  startTime, lastYaw, lastArea;
    private boolean                 strafeLocked, throttleLocked, noTarget;

    public AlignToTape(PhotonVision photonVision, DriveBase driveBase)
    {
        this.photonVision = photonVision;

        strafeController.setOutputRange(-maxSpeed, maxSpeed);
        throttleController.setOutputRange(-maxSpeed, maxSpeed);
        throttleController.setOutputRange(-maxSpeed, maxSpeed);

        // We want zero yaw to target as reported by PV.
        strafeController.setSetpoint(0);
        strafeController.setTolerance(.5);

        // We want to be close enough so that target is .80% of field
        // of vision as reported by PV.
        throttleController.setSetpoint(.80);
        throttleController.setTolerance(.05);

        rotateController.setSetpoint(0);
        rotateController.setTolerance(.5);

        this.driveBase = driveBase;

        addRequirements(driveBase);
    }
    
    @Override
    public void initialize()
    {
        Util.consoleLog();

        // Select the reflective tape pipeline.
        photonVision.selectPipeline(0);

        photonVision.setLedMode(VisionLEDMode.kOn);

        startTime = Util.timeStamp();

        strafeController.reset();
        throttleController.reset();
        rotateController.reset();

        noTarget = strafeLocked = throttleLocked = false;
        lastArea = lastYaw = Double.NaN;

        //driveBase.setBrakeMode(true);

        SmartDashboard.putBoolean("AutoTarget", true);
        SmartDashboard.putBoolean("TargetLocked", false);
    }

    @Override
    public void execute()
    {
        // Delay first test for targets to allow PV to get going after we turn on LED
        // and process the next camera image. Otherwise we call getLatestResult before
        // there is one available and result is null.

        if (Util.getElaspedTime(startTime) < 0.75) return;

        result = photonVision.getLatestResult();

        if (result.hasTargets())
        {
            PhotonTrackedTarget target = result.getBestTarget();

            lastYaw = target.getYaw();

            double strafe = strafeController.calculate(lastYaw);
    
            if (strafeController.onTarget()) 
            {
                strafe = 0;
                strafeLocked = true;
            }
    
            SmartDashboard.putNumber("Strafe Speed", strafe);
    
            Util.consoleLog("yaw=%.2f  strafe=%.4f", lastYaw, strafe);
            
            lastArea = target.getArea();

            double throttle = throttleController.calculate(lastArea);

            if (throttleController.onTarget()) 
            {
                throttle = 0;
                throttleLocked =  true;
            }

            SmartDashboard.putNumber("Throttle Speed", throttle);

            double yaw = RobotContainer.navx.getYaw();

            double rotate = rotateController.calculate(yaw);

            Util.consoleLog("yaw=%.2f  strafe=%.4f  area=%.2f  throttle=%.4f", lastYaw, strafe, lastArea, throttle);

            driveBase.drive(-throttle, -strafe, rotate);
        } else 
            noTarget = true;    // No targets visible.
    }

    @Override
    public boolean isFinished()
    {
        return noTarget | (strafeLocked && throttleLocked);
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b, last Yaw=%.2f,  area=%.2f", interrupted, lastYaw, lastArea);

        driveBase.stop();

        //photonVision.setLedMode(VisionLEDMode.kOff);

        SmartDashboard.putBoolean("AutoTarget", false);

        if (strafeLocked && throttleLocked) SmartDashboard.putBoolean("TargetLocked", true);
    }
}
