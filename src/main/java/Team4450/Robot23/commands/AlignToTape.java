package Team4450.Robot23.commands;

import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.DriveBase;
import Team4450.Robot23.subsystems.PhotonVision;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AlignToTape extends CommandBase
{
    private PhotonVision            photonVision;
    private PhotonPipelineResult    result;
    private DriveBase               driveBase;
    private SynchronousPID          controller = new SynchronousPID("AlignToTape", 0.1, 0, 0);
    private final double            maxPower = .10, tolerance = 1.0;


    public AlignToTape(PhotonVision photonVision, DriveBase driveBase)
    {
        this.photonVision = photonVision;

        controller.setOutputRange(-maxPower, maxPower);

        controller.setSetpoint(0);

        controller.setTolerance(tolerance);

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

        controller.reset();

        SmartDashboard.putBoolean("AutoTarget", true);
        SmartDashboard.putBoolean("TargetLocked", false);
    }

    @Override
    public void execute()
    {
        result = photonVision.getLatestResult();

        if (result.hasTargets())
        {
            PhotonTrackedTarget target = result.getBestTarget();

            double speed = controller.calculate(target.getYaw());

            SmartDashboard.putNumber("Align Speed", speed);

            //driveBase.drive(0, speed, 0);
        }
    }

    @Override
    public boolean isFinished()
    {
        return result.hasTargets();
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);

        driveBase.stop();

        photonVision.setLedMode(VisionLEDMode.kOff);

        SmartDashboard.putBoolean("AutoTarget", false);
    }
}
