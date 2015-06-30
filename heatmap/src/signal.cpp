/*
 * signal.cpp
 *
 *  Created on: 10 Dec 2014
 *      Author: baueraff
 */

#include <heatmap/iwlib.h>
#include <ros/ros.h>
#include <heatmap/wifi_signal.h>
#include <heatmap/signal_service.h>

int g_skfd;
iwreq g_wrq;
wireless_info *g_info;
const char *g_wlan_dev;
heatmap::wifi_signal g_sig;



heatmap::wifi_signal get_signal(int skfd, iwreq wrq, const char *wlan_dev, wireless_info *info) {
	if(iw_get_stats(skfd, wlan_dev, &(info->stats), &info->range, info->has_range) >= 0)
		info->has_stats = 1;
	int quality = info->stats.qual.qual;
	// Get bit rate
	if(iw_get_ext(skfd, wlan_dev, SIOCGIWRATE, &wrq) >= 0) {
	    info->has_bitrate = 1;
	    memcpy(&(info->bitrate), &(wrq.u.bitrate), sizeof(info->bitrate));
	 }

	g_sig.link_quality = quality;

	 return g_sig;
}

heatmap::wifi_signal get_signal_average(int skfd, iwreq wrq, const char *wlan_dev, wireless_info *info) {
	const int measurements = 5;
	heatmap::wifi_signal avg_sig, sig;
	int qual_sum = 0;

	avg_sig = g_sig;

	for(int i = 0; i < measurements; i++) {
		sig = get_signal(skfd, wrq, wlan_dev, info);
		qual_sum += sig.link_quality;
		ros::Duration(0.02).sleep();
	}
	avg_sig.link_quality = qual_sum / measurements;

	return avg_sig;
}

bool srv_get_signal(heatmap::signal_service::Request &req, heatmap::signal_service::Response &res)
{
	res.signal = get_signal_average(g_skfd, g_wrq, g_wlan_dev, g_info);

	return true;
}

int main(int argc, char** argv) {
  int goterr = 0;
  iwreq	wrq;

  if(argc < 2) {
	  ROS_ERROR("You must specify the wlan device name. (e.g. wlan0)");
	  return -1;
  }

  char* wlan_dev = argv[1];

  ros::init(argc, argv, "signal_meter");
  ros::NodeHandle nh;

  ros::Publisher signal_pub = nh.advertise<heatmap::wifi_signal>("signal", 1000);
  ros::ServiceServer service = nh.advertiseService("get_wifi_signal", srv_get_signal);


  ros::Rate loop_rate(10);

  if((g_skfd = iw_sockets_open()) < 0)
  {
      ROS_ERROR("socket error");
      return -1;
  }

  g_info = new wireless_info();

  if(iw_get_basic_config(g_skfd, wlan_dev, &(g_info->b)) < 0)
    {
      /* If no wireless name : no wireless extensions */
      /* But let's check if the interface exists at all */
      struct ifreq ifr;

      strncpy(ifr.ifr_name, wlan_dev, IFNAMSIZ);
      if(ioctl(g_skfd, SIOCGIFFLAGS, &ifr) < 0) {
    	ROS_ERROR("device doesn't exist");
	    return(-ENODEV);
      }
      else {
    	ROS_ERROR("operation not supported");
	    return(-EOPNOTSUPP);
      }
    }

  g_sig.essid = g_info->b.essid;

  if(iw_get_range_info(g_skfd, wlan_dev, &(g_info->range)) >= 0)
         g_info->has_range = 1;
  g_sig.link_quality_max = g_info->range.max_qual.qual;

  g_wlan_dev = wlan_dev;

  ros::spin();

  delete g_info;
  return 0;
}
