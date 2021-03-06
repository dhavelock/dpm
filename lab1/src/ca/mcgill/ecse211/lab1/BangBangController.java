package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {
	
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  
  private int filterControl;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
    
    this.filterControl = 0;
  }

  @Override
  public void processUSData(int distance) {
	// TODO: process a movement based on the us distance passed in (BANG-BANG style)    

    if (distance >= 255 && filterControl < FILTER_OUT) {
        // bad value, do not set the distance var, however do increment the
        // filter value
        filterControl++;
    } else if (distance >= 255) {
        // We have repeated large values, so there must actually be nothing
        // there: leave the distance alone
        this.distance = distance;
    } else {
        // distance went below 255: reset filter and leave
        // distance alone.
        filterControl = 0;
        this.distance = distance;
    }
    
    int distError = this.distance - this.bandCenter;
    
    if (Math.abs(distError) <= this.bandwidth) { // within desired bandwidth
    	WallFollowingLab.leftMotor.setSpeed(this.motorHigh);
        WallFollowingLab.rightMotor.setSpeed(this.motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
        
    } else if (this.distance <= 15) { // extremely too close
    	WallFollowingLab.leftMotor.setSpeed(this.motorLow);
        WallFollowingLab.rightMotor.setSpeed(this.motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.backward();
        
    } else if (distError > 0) { // too far
    	WallFollowingLab.leftMotor.setSpeed(this.motorLow + 30);
        WallFollowingLab.rightMotor.setSpeed(this.motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
        
    } else if (distError < 0) { // too close
    	WallFollowingLab.leftMotor.setSpeed(this.motorHigh);
        WallFollowingLab.rightMotor.setSpeed(this.motorLow);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
    }
    
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
