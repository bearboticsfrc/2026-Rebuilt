package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Copilot {

  public enum Buttons {
    ROLLER_IDLE(0),
    ROLLER_FWDSLOW(1),
    ROLLER_FWDFAST(2),
    ROLLER_REVSLOW(3),
    ROLLER_REVFAST(4),
    SLIDER_IDLE(5),
    SLIDER_IN(6),
    SLIDER_MIDDLE(7),
    SLIDER_OUT(8),
    SLIDER_CALIBRATE(9),
    SPINDEXER_IDLE(10),
    SPINDEXER_FWDSLOW(11),
    SPINDEXER_FWDFAST(12),
    SPINDEXER_REVSLOW(13),
    SPINDEXER_REVFAST(14),
    KICKER_IDLE(15),
    KICKER_FWDSLOW(16),
    KICKER_FWDFAST(17),
    KICKER_REVSLOW(18),
    KICKER_REVFAST(19),
    FLYWHEEL_IDLE(20),
    FLYWHEEL_500RPM(21),
    FLYWHEEL_1200RPM(22),
    FLYWHEEL_3700RPM(23),
    HOOD_IDLE(25),
    //0_25 = .25
    HOOD_0_25(26),
    HOOD_0_5(27),
    HOOD_0_75(28),
    HOOD_1(29),
    BRAKE_MODE(30);

    public final int value;

    Buttons(int value) {
      this.value = value;
    }

    public static int getValue(Buttons b) {
      return b.value;
    }
  }

  public enum POV {
    TURRET_0(0),
    TURRET_45(45),
    TURRET_90(90),
    TURRET_135(135),
    TURRET_180(180),
    TURRET_225(225),
    TURRET_270(270),
    TURRET_315(315),
    TURRET_IDLE(360);

    public final int angle;

    POV(int angle) {
      this.angle = angle;
    }

    public static int getAngle(POV p) {
      return p.angle;
    }
  }

  private static CommandGenericHID copilot = new CommandGenericHID(0);

  public static Trigger turret0Degrees() {
    return copilot.pov(POV.getAngle(POV.TURRET_0));
  }

  public static Trigger turret45Degrees() {
    return copilot.pov(POV.getAngle(POV.TURRET_45));
  }

  public static Trigger turret90Degrees() {
    return copilot.pov(POV.getAngle(POV.TURRET_90));
  }

  public static Trigger turret135Degrees() {
    return copilot.pov(POV.getAngle(POV.TURRET_135));
  }

  public static Trigger turret180Degrees() {
    return copilot.pov(POV.getAngle(POV.TURRET_180));
  }

  public static Trigger turret225Degrees() {
    return copilot.pov(POV.getAngle(POV.TURRET_225));
  }

  public static Trigger turret270Degrees() {
    return copilot.pov(POV.getAngle(POV.TURRET_270));
  }

  public static Trigger turret315Degrees() {
    return copilot.pov(POV.getAngle(POV.TURRET_315));
  }

  public static Trigger turretIdle() {
    return copilot.pov(POV.getAngle(POV.TURRET_IDLE));
  }

  public static Trigger rollerIdle() {
    return copilot.button(Buttons.getValue(Buttons.ROLLER_IDLE));
  }

  public static Trigger rollerFwdSlow() {
    return copilot.button(Buttons.getValue(Buttons.ROLLER_FWDSLOW));
  }

  public static Trigger rollerFwdFast() {
    return copilot.button(Buttons.getValue(Buttons.ROLLER_FWDFAST));
  }

  public static Trigger rollerRevSlow() {
    return copilot.button(Buttons.getValue(Buttons.ROLLER_REVSLOW));
  }

  public static Trigger rollerRevFast() {
    return copilot.button(Buttons.getValue(Buttons.ROLLER_REVFAST));
  }

  public static Trigger sliderIdle() {
    return copilot.button(Buttons.getValue(Buttons.SLIDER_IDLE));
  }

  public static Trigger sliderIn() {
    return copilot.button(Buttons.getValue(Buttons.SLIDER_IN));
  }

  public static Trigger sliderMiddle() {
    return copilot.button(Buttons.getValue(Buttons.SLIDER_MIDDLE));
  }

  public static Trigger sliderOut() {
    return copilot.button(Buttons.getValue(Buttons.SLIDER_OUT));
  }

  public static Trigger sliderCalibrate() {
    return copilot.button(Buttons.getValue(Buttons.SLIDER_CALIBRATE));
  }

    public static Trigger spindexerIdle() {
    return copilot.button(Buttons.getValue(Buttons.SPINDEXER_IDLE));
  }

  public static Trigger spindexerFwdSlow() {
    return copilot.button(Buttons.getValue(Buttons.SPINDEXER_FWDSLOW));
  }
  
  public static Trigger spindexerFwdFast() {
    return copilot.button(Buttons.getValue(Buttons.SPINDEXER_FWDFAST));
  }

  public static Trigger spindexerRevSlow() {
    return copilot.button(Buttons.getValue(Buttons.SPINDEXER_REVSLOW));
  }

  public static Trigger spindexerRevFast() {
    return copilot.button(Buttons.getValue(Buttons.SPINDEXER_REVFAST));
  }

  public static Trigger kickerIdle() {
    return copilot.button(Buttons.getValue(Buttons.KICKER_IDLE));
  }

  public static Trigger kickerFwdSlow() {
    return copilot.button(Buttons.getValue(Buttons.KICKER_FWDSLOW));
  }

  public static Trigger kickerFwdFast() {
    return copilot.button(Buttons.getValue(Buttons.KICKER_FWDFAST));
  }

  public static Trigger kickerRevSlow() {
    return copilot.button(Buttons.getValue(Buttons.KICKER_REVSLOW));
  }

  public static Trigger kickerRevFast() {
    return copilot.button(Buttons.getValue(Buttons.KICKER_REVFAST));
  }

   public static Trigger flywheelIdle() {
    return copilot.button(Buttons.getValue(Buttons.FLYWHEEL_IDLE));
  }

  public static Trigger flywheel500() {
    return copilot.button(Buttons.getValue(Buttons.FLYWHEEL_500RPM));
  }

  public static Trigger flywheel1200() {
    return copilot.button(Buttons.getValue(Buttons.FLYWHEEL_1200RPM));
  }

  public static Trigger flywheel3700() {
    return copilot.button(Buttons.getValue(Buttons.FLYWHEEL_3700RPM));
  }

  public static Trigger hoodIdle() {
    return copilot.button(Buttons.getValue(Buttons.HOOD_IDLE));
  }

  public static Trigger hood0_25() {
    return copilot.button(Buttons.getValue(Buttons.HOOD_0_25));
  }

  public static Trigger hood0_5() {
    return copilot.button(Buttons.getValue(Buttons.HOOD_0_25));
  }

  public static Trigger hood0_75() {
    return copilot.button(Buttons.getValue(Buttons.HOOD_0_75));
  }

  public static Trigger hood1() {
    return copilot.button(Buttons.getValue(Buttons.HOOD_1));
  }

  public static Trigger brakeMode() {
    return copilot.button(Buttons.getValue(Buttons.BRAKE_MODE));
  }

}
