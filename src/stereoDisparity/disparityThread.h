#include <iostream>
#include <string>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/dev/GazeControl.h>
#include <iCub/ctrl/Math.h>
#include "stereoCamera.h"

 
using namespace std; 
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
  
class disparityThread : public Thread
{
private:

   ImageOf<PixelRgb> *imageL;
   ImageOf<PixelRgb> *imageR;
   IplImage * imgL;   
   IplImage * imgR;
   IplImage disp;

   stereoCamera *stereo;
   Semaphore* mutex;

   string inputLeftPortName;
   string inputRightPortName;
   string outName;
   double angle;
   BufferedPort<ImageOf<PixelRgb> > imagePortInLeft;
   BufferedPort<ImageOf<PixelRgb> > imagePortInRight;
   BufferedPort<ImageOf<PixelBgr> > outPort;

   Port *commandPort;
   string dir;

   PolyDriver* gazeCtrl;
   IGazeControl* igaze;
   Matrix H;
   Matrix tras;
   
   void getH();
   void printMatrixYarp(Matrix &A);
   void convert(Matrix &R, Mat& Rot);

public:

   disparityThread(string imageInLeft, string imageInRight, string outputPortName, string calibPath, Port* commPort);
   bool threadInit();     
   void threadRelease();
   void run(); 
   void onStop();
};

class updateCameraThread : public RateThread {

    private:
        stereoCamera *stereo;
        Semaphore * mutex;


    public:
        updateCameraThread(stereoCamera *camera, Semaphore * mut, int period);
        virtual void run();
        

};
