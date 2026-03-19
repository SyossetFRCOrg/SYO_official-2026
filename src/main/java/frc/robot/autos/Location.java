package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

public enum Location {

  // Field (Neutral) Locations
  NA("BNA", "RNA"),
  NB("BNB", "RNB"),
  NC("BNC", "RNC"),

  DEPOT("BDEPOT", "RDEPOT"),
  OUTPOST("BOUTPOST", "ROUTPOST"),

  TOWERLEFT("BTOWERLEFT", "RTOWERLEFT"),
  TOWERMIDLEFT("BTOWERMIDLEFT", "RTOWERMIDLEFT"),
  TOWERMIDRIGHT("BTOWERMIDRIGHT", "RTOWERMIDRIGHT"),
  TOWERRIGHT("BTOWERRIGHT", "RTOWERRIGHT"),
  
  S1("BS1", "RS1"),
  S2("BS2", "RS2"),
  S3("BS3", "RS3"),

  FLSTART("BFLSTART", "RFLSTART"),
  LSTART("BLSTART", "RLSTART"),
  MIDSTART("BMIDSTART", "RMIDSTART"),
  RSTART("BRSTART", "RRSTART"),
  FRSTART("BFRSTART", "RFRSTART"),

  LTRENCH("BLTRENCH", "RLTRENCH"),
  RTRENCH("BRTRENCH", "RRTRENCH"),

  LBUMP("BLBUMP","RLBUMP"),
  RBUMP("BRBUMP","RRBUMP"),

  FLDUMMYSHOOT("BFLDUMMYSHOOT", "RFLDUMMYSHOOT"),
  LDUMMYSHOOT("BLDUMMYSHOOT","RLDUMMYSHOOT"),
  FRDUMMYSHOOT("BFRDUMMYSHOOT","RFRDUMMYSHOOT"),
  RDUMMYSHOOT("BRDUMMYSHOOT","RRDUMMYSHOOT"),

  NONE("NONE", "NONE");

  private final String blueName, redName;

  private Location(String blueName, String redName) {
    this.blueName = blueName;
    this.redName = redName;
  }

  public String getAllianceName() {
    return FieldConstants.getAlliance() == Alliance.Blue ? blueName : redName;
  }
}
