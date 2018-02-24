import processing.serial.*;
import grafica.*;

// gps coordinates, rotation rate, signal strength
// elevation, velocity, air pressure, temp (graph of values in addition to printed values)
// save data to file

Serial myPort;  // The serial port
PrintWriter output; // The data file

PFont f;

float ax;
float ay;
float az;
float roll;
float pitch;
float heading;
float altitudeBarometer;
float altitudeGPS;
float velocity;
float velocityIntegrated;
float latitude;
float longitude;
float time;

float[] axList = new float[0];
float[] ayList= new float[0];
float[] azList= new float[0];
float[] rollList= new float[0];
float[] pitchList= new float[0];
float[] headingList= new float[0];
float[] altitudeBarometerList= new float[0];
float[] altitudeGPSList= new float[0];
float[] velocityList= new float[0];
float[] velocityIntegratedList= new float[0];
float[] latitudeList= new float[0];
float[] longitudeList= new float[0];
float[] timeList= new float[0];

PImage imgTitle;
PImage imgAlt;
PImage imgVel;
PImage imgAcc;
PImage imgMapRibbon;
int deltatime;
int deltatimeTransmission;

//String inByte = "";
String inByte;
String inByteNew;
String inByteFirst;
String inByteFinal;
String parameters;
String[][] parametersList;

void setup(){
  size(displayWidth,displayHeight);
  imgTitle = loadImage("Rocket Logo.jpg");
  imgAlt = loadImage("Altitude Gauge.jpg");
  imgVel = loadImage("Velocity Gauge.jpg");
  imgAcc = loadImage("Acceleration Gauge.jpg");
  imgMapRibbon = loadImage("Rocket Map.jpg");
  myPort = new Serial(this, Serial.list()[3], 9600);
  inByteNew = "";
  f = createFont("Anita semi-square", 50, true);
  inByteFirst = myPort.readString();
  deltatime = 50;
  velocityIntegrated = 0;
  deltatimeTransmission = 100;
  
  output = createWriter("data.txt");
}

void draw(){
  background(#E7E7E7);
  image(imgTitle, displayWidth/2-displayWidth/4.5, 0, displayWidth/2.25, displayWidth/9);
  image(imgAlt, 0, displayWidth/9, (displayHeight-displayWidth/9)/3*638/738,(displayHeight-displayWidth/9)/3);
  image(imgVel, 0, displayWidth/9+(displayHeight-displayWidth/9)/3, (displayHeight-displayWidth/9)/3*638/738,(displayHeight-displayWidth/9)/3);
  image(imgAcc, 0, displayWidth/9+(displayHeight-displayWidth/9)/3*2, (displayHeight-displayWidth/9)/3*638/738,(displayHeight-displayWidth/9)/3);
  image(imgMapRibbon, (displayHeight-displayWidth/9)/3*638/738, displayWidth/9, displayWidth - (displayHeight-displayWidth/9)/3*638/738, (displayWidth - (displayHeight-displayWidth/9)/3*638/738)*73/1480);
  fill(255);
  stroke(255);
  rect((displayHeight-displayWidth/9)/3*638/738+75, 25 + displayWidth/9 + (displayWidth - (displayHeight-displayWidth/9)/3*638/738)*73/1480, -50 + displayWidth - (displayHeight-displayWidth/9)/3*638/738, -50 + displayHeight -displayWidth/9 + (displayWidth - (displayHeight-displayWidth/9)/3*638/738)*73/1480); 
  while (myPort.available() > 0) {
    
    //Reading and String Checking
    inByte = myPort.readString();
    println(inByte);
    inByte = inByteNew + inByte;
    if (inByte.indexOf("*") != -1){
      while(inByte.indexOf("&") == -1){
        delay(deltatime);
        String inByteCutNew = myPort.readString();
        inByte = inByte + inByteCutNew;
      }
      String inByteCut = inByte.substring(inByte.indexOf("*"),inByte.indexOf("&")+1);
      inByteFinal = inByteCut;
    }
    
    println(inByteFinal);
    
    //Cutting Input String into parameters
    
    while(inByteFinal.indexOf("*")==-1){
      draw();
    }
    println(inByteFinal);
    parameters = inByteFinal.substring(inByteFinal.indexOf("*")+1,inByteFinal.indexOf("&"));
    output.println(parameters);
    parametersList = matchAll(parameters,"#(.*?)#");
    
    roll = float(parametersList[0][1]);
    pitch = float(parametersList[1][1]);
    heading = float(parametersList[2][1]);
    /*ax = float(parametersList[0][1]);
    ay = float(parametersList[1][1]);
    az = float(parametersList[2][1]);
    roll = float(parametersList[3][1]);
    pitch = float(parametersList[4][1]);
    heading = float(parametersList[5][1]);
    altitudeBarometer = float(parametersList[6][1]);
    altitudeGPS = float(parametersList[7][1]);
    velocity = float(parametersList[8][1]);
    latitude = float(parametersList[9][1]);
    longitude = float(parametersList[10][1]);
    time = float(parametersList[11][1]);
    velocityIntegrated = velocityIntegrated + sqrt(ax*ax+ay*ay+az*az)*deltatime/1000; */
    
    fill(0);
    textFont(f,25);
    textAlign(LEFT);
    text("ax: " + ax, 60, displayWidth/9+(displayHeight-displayWidth/9)/3*2+125);
    text("ay: " + ay, 60, displayWidth/9+(displayHeight-displayWidth/9)/3*2+150);
    text("az: " + az, 60, displayWidth/9+(displayHeight-displayWidth/9)/3*2+175);
    
    
    text("GPS: " + velocity, 60, displayWidth/9 + (displayHeight-displayWidth/9)/3+140);
    text("INT: " + nfc(velocityIntegrated,2), 60, displayWidth/9 + (displayHeight-displayWidth/9)/3+170);
    
    text("ROLL: " + roll, displayWidth/4-40, displayWidth/9 + (displayHeight-displayWidth/9)/4-40);
    text("PITCH: " + pitch, displayWidth/4-40, displayWidth/9 + (displayHeight-displayWidth/9)/4);
    text("HEADING: " + heading, displayWidth/4-40, displayWidth/9 + (displayHeight-displayWidth/9)/4+40);
    
    text("GPS: " + altitudeGPS, 60, displayWidth/9 +140);
    text("BAR: " + altitudeBarometer, 60, displayWidth/9 +170);
    fill(255);
    text(nfs(floor(time/60),2,0)+":"+nfs(time%60,2,2),displayWidth*4/5+40,198);
    text(nfs(latitude,2,6)+"°N " + nfs(longitude,2,6) + "°W", displayWidth/3+100, 198);
    
    axList = (float[])append(axList,ax);
    ayList = (float[])append(ayList,ay);
    azList = (float[])append(azList,az);
    rollList = (float[])append(rollList,roll);
    pitchList = (float[])append(pitchList,pitch);
    headingList = (float[])append(headingList,heading);
    altitudeBarometerList = (float[])append(altitudeBarometerList,altitudeBarometer);
    altitudeGPSList = (float[])append(altitudeGPSList,altitudeGPS);
    velocityList = (float[])append(velocityList,velocity);
    latitudeList = (float[])append(latitudeList,latitude);
    longitudeList = (float[])append(longitudeList,longitude);
    timeList = (float[])append(timeList,time);
    velocityIntegratedList = (float[])append(velocityIntegratedList,velocityIntegrated);
  } 
  
  //PLOTS DRAWER
  float[] firstPlotPos = new float[] {0, 0};
  float[] panelDim = new float[] {400, 225};
  float[] margins = new float[] {60,displayWidth*1/3, displayHeight/3, 60};
    // Create four plots to represent the 4 panels
  GPlot plot1 = new GPlot(this);
  plot1.setPos(firstPlotPos);
  plot1.setMar(0, margins[1], margins[2], 0);
  plot1.setDim(panelDim);
  plot1.setAxesOffset(0);
  plot1.setTicksLength(-4);
  plot1.getXAxis().setDrawTickLabels(false);

  GPlot plot2 = new GPlot(this);
  plot1.setTitleText("Velocity & Altitude vs. Time");
  plot2.setPos(firstPlotPos[0] + margins[1] + panelDim[0], firstPlotPos[1]);
  plot2.setMar(0, 0, margins[2], margins[3]);
  plot2.setDim(panelDim);
  plot2.setAxesOffset(0);
  plot2.setTicksLength(-4);
  plot2.getXAxis().setDrawTickLabels(false);
  plot2.getYAxis().setDrawTickLabels(true);

  GPlot plot3 = new GPlot(this);
  plot3.setPos(firstPlotPos[0], firstPlotPos[1] + margins[2] + panelDim[1]);
  plot3.setMar(margins[0], margins[1], 0, 0);
  plot3.setDim(panelDim);
  plot3.setAxesOffset(0);
  plot3.setTicksLength(-4);

  GPlot plot4 = new GPlot(this);
  plot4.setPos(firstPlotPos[0] + margins[1] + panelDim[0], firstPlotPos[1] + margins[2] + panelDim[1]);
  plot4.setMar(margins[0], 0, 0, margins[3]);
  plot4.setDim(panelDim);
  plot4.setAxesOffset(0);
  plot4.setTicksLength(-4);
  plot4.getYAxis().setDrawTickLabels(true);

  // Prepare the points for the four plots
  int nPoints = timeList.length;
  GPointsArray points1 = new GPointsArray(nPoints);
  GPointsArray points2 = new GPointsArray(nPoints);
  GPointsArray points3 = new GPointsArray(nPoints);
  GPointsArray points4 = new GPointsArray(nPoints);

  for (int i = 0; i < nPoints; i++) {
    points1.add(timeList[i], velocityList[i]);
    points2.add(timeList[i], velocityIntegratedList[i]);
    points3.add(timeList[i], altitudeGPSList[i]);
    points4.add(timeList[i], altitudeBarometerList[i]);
  }  

  // Set the points, the title and the axis labels
  plot1.setPoints(points1);
  plot2.setTitleText("VELOCITY & ALTITUDE VS. TIME");
  plot2.getTitle().setRelativePos(0.25);
  plot2.getTitle().setTextAlignment(LEFT);
  plot2.getYAxis().setAxisLabelText("Velocity (m/s)");

  plot2.setPoints(points2);
  

  plot3.setPoints(points3);
  plot3.getYAxis().setAxisLabelText("Altitude (m)");
  plot3.setInvertedYScale(true);

  plot4.setPoints(points4);
  plot4.getYAxis().setAxisLabelText("Position (m)");
  plot4.getXAxis().setAxisLabelText("Time (sec)");
  plot4.setInvertedYScale(true);

  // Draw the plots

  plot2.beginDraw();
  plot2.drawTitle();
  plot2.drawBox();
  plot2.drawXAxis();
  plot2.drawYAxis();
  plot2.drawTopAxis();
  plot2.drawRightAxis();
  plot2.drawPoints();
  plot2.drawLines();
  plot1.drawPoints();
  plot1.drawLines();
  plot2.endDraw();

  plot4.beginDraw();
  plot4.drawBox();
  plot4.drawXAxis();
  plot4.drawYAxis();
  plot4.drawTopAxis();
  plot4.drawRightAxis();
  plot4.drawPoints();
  plot3.drawPoints();
  plot3.drawLines();
  plot4.drawLines();
  plot4.endDraw();

  delay(deltatime);
}