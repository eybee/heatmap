/* Copyright 2015 Institute of Digital Communication Systems - Ruhr-University Bochum
 * Author: Adrian Bauer
 *
 * This program is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation;
 * either version 3 of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with this program;
 * if not, see <http://www.gnu.org/licenses/>.
 *
 **/


#include <ros/ros.h>
#include <heatmap/wifi_signal.h>
#include <heatmap/signal_service.h>
#include <heatmap/iwlib.h>

namespace heatmap
{
/**
* @brief Class that reads the signal quality of a wifi device and serves it
* 1. As a topic
* 2. As a service
* The actual core functionality is taken from the iwconfig source code
*/
class Signal
{
private:
  int wifi_skfd;
  iwreq wifi_wrq;
  wireless_info *wifi_info;
  const char *wifi_dev;
  heatmap::wifi_signal wifi_sig;
  ros::NodeHandle nh;

  /**
  * @brief Gets the signal quality property of the wifi device
  * @return An updated wifi_signal type
  */
  wifi_signal getSignal()
  {
    if (iw_get_stats(wifi_skfd, wifi_dev, &(wifi_info->stats), &wifi_info->range, wifi_info->has_range) >= 0)
      wifi_info->has_stats = 1;
    int quality = wifi_info->stats.qual.qual;
    // Get bit rate
    if (iw_get_ext(wifi_skfd, wifi_dev, SIOCGIWRATE, &wifi_wrq) >= 0)
    {
      wifi_info->has_bitrate = 1;
      memcpy(&(wifi_info->bitrate), &(wifi_wrq.u.bitrate), sizeof(wifi_info->bitrate));
    }

    wifi_sig.link_quality = quality;

    return wifi_sig;
  }

  /**
  * @brief Calculates an average of multiple signal quality measurements
  * @return An updated wifi_signal type
  */
  wifi_signal getSignalAverage()
  {
    const int measurements = 5;
    heatmap::wifi_signal avg_sig, sig;
    int qual_sum = 0;

    avg_sig = wifi_sig;

    for (int i = 0; i < measurements; i++)
    {
      sig = getSignal();
      qual_sum += sig.link_quality;
      ros::Duration(0.02).sleep();
    }
    avg_sig.link_quality = qual_sum / measurements;

    return avg_sig;
  }

  bool srvGetSignal(heatmap::signal_service::Request &req, heatmap::signal_service::Response &res)
  {
    res.signal = getSignalAverage();

    return true;
  }

  /**
  * @brief Fills in basic information like the essid and the maximum signal quality
  */
  bool openDevice()
  {
    if ((wifi_skfd = iw_sockets_open()) < 0)
    {
      ROS_ERROR("socket error");
      return false;
    }

    if (iw_get_basic_config(wifi_skfd, wifi_dev, &(wifi_info->b)) < 0)
    {
      /* If no wireless name : no wireless extensions */
      /* But let's check if the interface exists at all */
      struct ifreq ifr;

      strncpy(ifr.ifr_name, wifi_dev, IFNAMSIZ);
      int ret = ioctl(wifi_skfd, SIOCGIFFLAGS, &ifr);
      if (ret < 0)
      {
        ROS_ERROR("device doesn't exist");
        return false;
      }
      else
      {
        ROS_ERROR("operation not supported");
        return false;
      }
    }

    wifi_sig.essid = wifi_info->b.essid;

    if (iw_get_range_info(wifi_skfd, wifi_dev, &(wifi_info->range)) >= 0)
      wifi_info->has_range = 1;

    wifi_sig.link_quality_max = wifi_info->range.max_qual.qual;

    return true;

  }

public:
  Signal(const char* wlan_dev) :
      nh()
  {
    wifi_dev = wlan_dev;
    wifi_info = new wireless_info();
    ros::Rate loop_rate(10.0);

    if(!openDevice())
    {
      return;
    }

    ros::Publisher signal_pub = nh.advertise<heatmap::wifi_signal>("signal", 1000);
    ros::ServiceServer service = nh.advertiseService("get_wifi_signal", &Signal::srvGetSignal, this);

    while(nh.ok())
    {
      wifi_signal sig = getSignal();
      signal_pub.publish(sig);
      loop_rate.sleep();
      ros::spinOnce();
    }
  }

  ~Signal()
  {
    delete wifi_info;
  }
};
}

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    ROS_ERROR("You must specify the wlan device name. (e.g. wlan0)");
    return -1;
  }
  char* wlan_dev = argv[1];

  ros::init(argc, argv, "signal_meter");

  heatmap::Signal signal(wlan_dev);

  return 0;
}
