package ca.mcgill.ecse211.Navigator;

public class NavigatorObstacle extends Thread implements UltrasonicController {

  private static final int FILTER_OUT = 20;
  
  private int distance;
  private int filterControl;
  
  public NavigatorObstacle() {
    this.distance = 0;
    this.filterControl = 0;
  }
  
  public int getDistance() {
    return distance;
  }
  
  @Override
  public void processUSData(int distance) {
    
    // filter distance
    if (distance >= 255 && filterControl < FILTER_OUT) {
      filterControl++;
    } else if (distance >= 255) {
      this.distance = distance;
    } else {
      filterControl = 0;
      this.distance = distance;
    }
        
    
  }
  
  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
