import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TrajectoryCalculator.ShotCalculator;
import frc.spectrumLib.CachedDouble;

public class ShootHood extends SubsystemBase{
    private final PositionVoltage m_positionRequest = new PositionVoltage(0);
    private final CachedDouble cachedVoltage;
    private final TalonFX shooterHood; 
    
    public void ShootHood(){
        shooterHood = new TalonFX(0); // add actual ID

        //shooter config
        TalonFXConfiguration shooterHoodConfig = new TalonFXConfiguration();
        
        shooterHoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterHoodConfig.Slot0.kP = 0;
        shooterHoodConfig.Slot0.kI = 0;
        shooterHoodConfig.Slot0.kD = 0;

        //apply config
        shooterHood.getConfigurator().apply(shooterHoodConfig);

    }
    //sets hood angle to specified angle
    public Command setHoodAngle(Angle hoodAngle){
        return Commands.run(()-> shooterHood.setControl(m_positionRequest.withPosition(hoodAngle)), this);
    }

    @Logged
    public double getVoltage() {
        return cachedVoltage.getAsDouble();
  }

}