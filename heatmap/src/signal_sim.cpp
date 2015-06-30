/*
 * signal_sim.cpp
 *
 *  Created on: Jun 22, 2015
 *      Author: ros
 */

#include <ros/ros.h>
#include <heatmap/wifi_signal.h>
#include <heatmap/signal_service.h>

heatmap::wifi_signal ws;

bool srvGetSignal(heatmap::signal_service::Request &req, heatmap::signal_service::Response &res)
{
  res.signal = ws;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "signal_meter_sim");

  ros::NodeHandle nh;

  ros::Publisher signal_pub = nh.advertise<heatmap::wifi_signal>("signal", 1000);
  ros::ServiceServer service = nh.advertiseService("get_wifi_signal", srvGetSignal);

  ros::Rate loop_rate(10);

  ws.essid = "Sim WIFI";
  ws.link_quality_max = 70;

  while(nh.ok())
  {
    ws.link_quality = rand() % 71;
    signal_pub.publish(ws);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
