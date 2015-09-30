#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <vector>

int i = 0;
char buf[4096];

class SimpleOpenNIViewer
 {
public:
 SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

 void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
 {
   //if (!viewer.wasStopped())
    //{
    // viewer.showCloud (cloud);
     pcl::PCDWriter w;
     sprintf (buf, "frame_%06d.pcd", i);
     w.writeBinaryCompressed (buf, *cloud);
     PCL_INFO ("Wrote a cloud with %zu (%ux%u) points in %s.\n",cloud->size (),     
cloud->width, cloud->height, buf);
     ++i;
    //}

 }

 void run ()
 {
   pcl::Grabber* interface = new pcl::ONIGrabber("file.oni",false,false); //set for trigger

   boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

   interface->registerCallback (f);

   interface->start ();

   while (!viewer.wasStopped())
       {
        interface->start();//to update each frame from the oni file 
        boost::this_thread::sleep (boost::posix_time::seconds (1));
       }
   PCL_INFO ("Successfully processed %d frames.\n", i);
   interface->stop ();
 }

 pcl::visualization::CloudViewer viewer;
 };

 int main ()
 {
   SimpleOpenNIViewer v;
   v.run();
   return 0;
 }
