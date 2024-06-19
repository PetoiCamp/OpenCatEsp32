enum ClipMode_t = {
  CLOSE = 0,
  OPEN,
  SNAP,
  CLAP,
};

Class ArmClip {
public:
  int restAngle;
  int snapThreshold;
  void clipAction(ClipMode_t mode) {
    switch (mode) {
      case CLOSE:
        {
          calibratedPWM(2, 120);
          break;
        }
      case OPEN:
        {
          calibratedPWM(2, 30);
          break;
        }
      case SNAP:
        {
          calibratedPWM(2, 20);
          calibratedPWM(2, 120);
          break;
        }
      case CLAP:
        {
          calibratedPWM(2, -120);
          calibratedPWM(2, 120);
          break;
        }
    }
  }
  void testClip(){ // m2 120 2 30 2 120 2 20 2 120 2 -120 2 120
      //i 1 -60 2 120
      //i 1 60 2 20
    clipAction(CLOSE);
    clipAction(OPEN);
    clipAction(SNAP);
    clipAction(CLAP);
  }
};
ArmClip armClip;
