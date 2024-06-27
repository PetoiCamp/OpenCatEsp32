enum ClipMode_t {
  CLOSE = 0,
  OPEN,
  SNAP,
  CLAP,
};

class ArmClip {
public:
  int restAngle;
  int snapThreshold;
  void clipAction(ClipMode_t mode) {
    switch (mode) {
      case CLOSE:
        {
          calibratedPWM(3, 120);
          break;
        }
      case OPEN:
        {
          calibratedPWM(3, 30);
          break;
        }
      case SNAP:
        {
          calibratedPWM(3, 20);
          calibratedPWM(3, 120);
          break;
        }
      case CLAP:
        {
          calibratedPWM(3, -120);
          calibratedPWM(3, 120);
          break;
        }
    }
  }
  void testClip() {  // m3 120 3 30 3 120 3 20 3 120 3 -120 3 120
    //i 1 -60 3 120
    //i 1 60 2 20
    clipAction(CLOSE);
    clipAction(OPEN);
    clipAction(SNAP);
    clipAction(CLAP);
  }
};
ArmClip armClip;
