import ipcapture.*;

IPCapture cam;
PImage camera_image;

String ROS_TOPIC = "/input_image";

void setup() {
  size(640,480);
  cam = new IPCapture(this, "http://localhost:8080/stream?topic="+ROS_TOPIC+"&height=480&widht=640&type=mjpeg", "", "");
  cam.start();
  
  // this works as well:
  
  // cam = new IPCapture(this);
  // cam.start("url", "username", "password");
  
  // It is possible to change the MJPEG stream by calling stop()
  // on a running camera, and then start() it with the new
  // url, username and password.
}

void draw() {
  if (cam.isAvailable()) {
    cam.read();
    
    image(cam, 0, 0);
    // blur and threshold
    filter(BLUR, 6);
    filter(THRESHOLD, 0.5);
    
  }
}

void keyPressed() {
  if (key == ' ') {
    if (cam.isAlive()) cam.stop();
    else cam.start();
  }
}
