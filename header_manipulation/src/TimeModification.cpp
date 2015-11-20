#include <header_manipulation/TimeModification.h>

TimeModification::TimeModification(ros::Rate &rate) : update_rate_(rate) {
  private_nh_ = ros::NodeHandle("~");
  generic_sub_ = private_nh_.subscribe<topic_tools::ShapeShifter>("input", 10, &TimeModification::inputCB, this);

  reconf_server_.reset(new ReconfigureServer(private_nh_));
  reconf_server_->setCallback(boost::bind(&TimeModification::configCB, this, _1, _2));

  output_advertised_ = false;
  seq_counter_ = 0;
}

void TimeModification::configCB(Config &config, uint32_t level) {
  update_rate_ = ros::Rate(config.update_rate);
  time_offset_ = ros::Duration(config.time_offset_milliseconds/1000);
}

void TimeModification::inputCB(const topic_tools::ShapeShifter::ConstPtr &input) {
  ROS_DEBUG("in callback");
  if (!output_advertised_) {
    ROS_INFO("Output not advertised. Setting up publisher now.");
    ros::AdvertiseOptions opts("output", 1, input->getMD5Sum(), input->getDataType(), input->getMessageDefinition());
    generic_pub_ = private_nh_.advertise(opts);
    output_advertised_ = true;
  }
  std_msgs::Header header;
  uint8_t msg_buffer[input->size()];
  ros::serialization::OStream o_stream(msg_buffer, input->size());
  ros::serialization::IStream i_stream(msg_buffer, input->size());
  input->write(o_stream);
  header.seq = ((uint32_t *)msg_buffer)[0];
  header.stamp.sec = ((uint32_t *)msg_buffer)[1];
  header.stamp.nsec = ((uint32_t *)msg_buffer)[2];

  // TODO (fkunz) check if header data is consistent
  if ( abs((header.stamp - ros::Time::now()).toSec()) > 5.0 ) {
    ROS_WARN_THROTTLE(0.1, "Probably subscribed to a topic with a message type not containing a header.");
  }
  {
    header.stamp += time_offset_;
  }
  ((uint32_t *)msg_buffer)[0] = header.seq + seq_counter_++;
  ((uint32_t *)msg_buffer)[1] = header.stamp.sec;
  ((uint32_t *)msg_buffer)[2] = header.stamp.nsec;
  topic_tools::ShapeShifter output;
  output.read(i_stream);
  generic_pub_.publish(output);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "time_modification_node");
  ros::start();
  ros::Rate update_rate(60);

  TimeModification time_modification_node(update_rate);
  ros::spin();

  return 0;
}
