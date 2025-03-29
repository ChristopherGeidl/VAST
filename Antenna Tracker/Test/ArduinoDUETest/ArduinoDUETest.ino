double R = 6378137; //m Earth's radius
double e = 0.08181919; //Earth's eccentricity

struct Location{
  double latitude;
  double longitude;
  double altitude;
};

struct Angles {
  double azimuth;
  double elevation;
};

double deg2rad(double degree){return degree*PI/180.0;}
double rad2deg(double radian){return radian*180.0/PI;}

Location getAntennaLocation(){
  struct Location a;
  a.latitude = 0;
  a.longitude = 0;
  a.altitude = 0;
  return a;
}
Location getBalloonLocation(){
  struct Location b;
  b.latitude = 0;
  b.longitude = 0;
  b.altitude = 0;
  return b;
}

Angles getAngles(Location ant, Location ball){
  //Convert degrees to radians
  ant.latitude = deg2rad(ant.latitude);
  ant.longitude = deg2rad(ant.longitude);
  ball.latitude = deg2rad(ball.latitude);
  ball.longitude = deg2rad(ball.longitude);

  //Using Earth-Centered, Earth-Fixed (ECEF) coordinate system
  double Nb = R / sqrt(1 - e*e*sin(ball.latitude)*sin(ball.latitude));
  double Xb = (Nb + ball.altitude) * cos(ball.latitude) * cos(ball.longitude);
  double Yb = (Nb + ball.altitude) * cos(ball.latitude) * sin(ball.longitude);
  double Zb = (Nb * (1 - e*e) + ball.altitude) * sin(ball.latitude);

  double Na = R / sqrt(1 - e*e*sin(ant.latitude)*sin(ant.latitude));
  double Xa = (Na + ant.altitude) * cos(ant.latitude) * cos(ant.longitude);
  double Ya = (Na + ant.altitude) * cos(ant.latitude) * sin(ant.longitude);
  double Za = (Na * (1 - e*e) + ant.altitude) * sin(ant.latitude);

  //Relative Position in ECEF frame
  double r[3] = {(Xb-Xa), (Yb-Ya), (Zb-Za)};

  //Transform to ENU (East, North, Up) frame
  double m[3][3] = {
    {-sin(ant.longitude), cos(ant.longitude), 0},
    {-sin(ant.latitude)*cos(ant.longitude), -sin(ant.latitude)*sin(ant.longitude), cos(ant.latitude)},
    {cos(ant.latitude)*cos(ant.longitude), cos(ant.latitude)*sin(ant.longitude), sin(ant.latitude)}
  };

  double E = m[0][0]*r[0] + m[0][1]*r[1] + m[0][2]*r[2];
  double N = m[1][0]*r[0] + m[1][1]*r[1] + m[1][2]*r[2];
  double U = m[2][0]*r[0] + m[2][1]*r[1] + m[2][2]*r[2];

  struct Angles angles;
  angles.azimuth = rad2deg(atan2(E, N));
  angles.azimuth = fmod(angles.azimuth + 360, 360);//Ensure azimuth [0-360)

  double geometric_horizon_angle = rad2deg(atan2(U, sqrt(E*E+N*N)));
  double horizon_angle_correction = rad2deg(acos(R / (R + ant.altitude)));
  angles.elevation = geometric_horizon_angle - horizon_angle_correction;

  return angles;
}

//Run test cases
int close(double a, double b, double difference){
  double res = sqrt(sq(a-b));
  if(res >= difference){
    return 0;
  }
  return 1;
}
void test(char test_title[], double blat, double blong, double balt, double alat, double along, double aalt, double expected_elevation, double expected_azimuth){
  Serial.println(test_title);
  struct Location ant;
  ant.latitude = alat;
  ant.longitude = along;
  ant.altitude = aalt;
  struct Location ball;
  ball.latitude = blat;
  ball.longitude = blong;
  ball.altitude = balt;
  struct Angles expected;
  expected.azimuth = expected_azimuth;
  expected.elevation = expected_elevation;
  struct Angles angles = getAngles(ant, ball);
  if(close(expected.azimuth, angles.azimuth, 5) && close(expected.elevation, angles.elevation, 5)){
    Serial.println("PASS");
  }else{
    Serial.print("Expected(az,ev): ");
    Serial.print(expected.azimuth);
    Serial.print(", ");
    Serial.println(expected.elevation);
    Serial.print("Actual(az,ev): ");
    Serial.print(angles.azimuth);
    Serial.print(", ");
    Serial.println(angles.elevation);
  }
  Serial.println();
  //Current tolerance of +-5 degrees (normally well within)
  /*test("Same Location", 90, 0, 0, 90, 0, 0, 0, 0);
  test("Balloon Directly Above", 90, 0, 1000, 90, 0, 0, 90, 0);
  test("Balloon Directly Below (other side of earth)", -90, 0, 1000, 90, 0, 0, -90, 180);
  test("Balloon at Equator Prime Meridian, Antenna at North", 0, 0, 0, 90, 0, 0, -45, 180);
  test("Balloon at Equator 90E, Antenna at North", 0, 90, 0, 90, 0, 0, -45, 90);
  test("Balloon at Equator 90W, Antenna at North", 0, -90, 0, 90, 0, 0, -45, 270);
  test("Balloon 100m away 1000m up", 43.610385, -116.340367, 1000, 43.611345, -116.340319, 0, 84.29, 182);
  test("Balloon 100m away 0m up", 43.610385, -116.340367, 0, 43.611345, -116.340319, 0, 0, 182);
  test("Balloon 1000m away 1000m up", 43.575709, -116.334336, 1000, 43.583792, -116.334293, 0, 45, 180);
  test("Balloon 1000m away 0m up", 43.575709, -116.334336, 0, 43.583792, -116.334293, 0, 0, 180);
  test("Balloon 25000m away, 25000m up", 43.575709, -116.334336, 25000, 43.775270, -116.331695, 0, 45, 180);
  test("Balloon 25000m away, 25000m up, antenna 25000m up", 43.575709, -116.334336, 25000, 43.775270, -116.331695, 25000, -0.11, 180);
  test("Balloon 25000m away, 25000m up, antenna 12500m up", 43.575709, -116.334336, 25000, 43.775270, -116.331695, 12500, 26.5, 180);*/
}

struct Location balloon;
struct Location antenna;
struct Angles current_angles;
struct Angles new_angles;

void setup() {
  Serial.begin(9600);
  antenna = getAntennaLocation();//get antenna location
  //MUST BE SET UP TO FACE NORTH W/ PERPENDICULAR ELEVATION
  current_angles.azimuth = 0;
  current_angles.elevation = 0;
}

double azimuthGearRatio = 150/1;
double elevationGearRatio = 150/1;
double motorDegreeToSteps = 200/360; //(200 steps per revolution)/(360 degrees per revolution)

void loop() {
  balloon = getBalloonLocation();//get balloon location
  new_angles = getAngles(antenna, balloon);//find new angles
  //angleChange -> motorSpinChange -> motorSteps (round to int)
  int azimuthChange = round((current_angles.azimuth-new_angles.azimuth) * azimuthGearRatio * motorDegreeToSteps);
  int elevationChange = round((current_angles.elevation-new_angles.elevation) * elevationGearRatio * motorDegreeToSteps);
  //Move
  //moveAzimuth(azimuthChange);
  //moveElevation(elevationChange);
  //updateState
  current_angles = new_angles;
  delay(1000);
}
