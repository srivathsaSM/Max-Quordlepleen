package frc.robot.util.MotorWrapper;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SparkMaxWrapper extends frc.robot.util.MotorWrapper.Motor {
    private SparkMax sparkMax;
    private SparkMaxConfig sparkConfig;

    public SparkMaxWrapper(int ID, boolean inverted, boolean brushless, boolean brake) {
        super(ID, inverted, brushless, brake);

    }

    public void initialize() {
        if (brushless) {
            sparkMax = new SparkMax(ID, MotorType.kBrushless);
        } else {
            sparkMax = new SparkMax(ID, MotorType.kBrushed);
        }
        sparkConfig = new SparkMaxConfig();
        sparkConfig.idleMode(brake? IdleMode.kBrake : IdleMode.kCoast);
        sparkConfig.inverted(inverted);
        applyConfig();
    }

    public SparkMaxConfig getConfig() {
        return sparkConfig;
    }

    public SparkMax getSparkMax() {
        return sparkMax;
    }

    public void applyConfig() {
        sparkMax.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pid(double p, double i, double d) {
        sparkConfig.closedLoop.pid(p,i,d);
        applyConfig();
    }

    public void sva(double s, double v, double a) {
        sparkConfig.closedLoop.feedForward.kS(s).kV(v).kA(a);
        applyConfig();
    }

    public void g(double g) {
        sparkConfig.closedLoop.feedForward.kG(g);
        applyConfig();
    }

    public void cos(double cos) {
        sparkConfig.closedLoop.feedForward.kCos(cos);
        applyConfig();
    }

    public void cr(double cos, double cosRatio) {
        sparkConfig.closedLoop.feedForward.kCos(cos).kCosRatio(cosRatio);
        applyConfig();
    }

    public void posConvFactor(double factor) {
        sparkConfig.encoder.positionConversionFactor(factor);
        sparkConfig.encoder.velocityConversionFactor(factor/60.0);
        sparkConfig.absoluteEncoder.positionConversionFactor(factor);
        sparkConfig.absoluteEncoder.velocityConversionFactor(factor);
        applyConfig();
    }

}
