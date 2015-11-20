#include <header_manipulation/TimeModification.h>

TimeModification::TimeModification(const ros::NodeHandle &nh, ros::Rate &publish_rate) :
    publish_retry_rate_(publish_rate)
{
  private_nh_ = ros::NodeHandle("~");
  generic_sub_ = private_nh_.subscribe<topic_tools::ShapeShifter>("input", 10,
                                                                  &TimeModification::inputCB, this);

  reconf_server_.reset(new ReconfigureServer(private_nh_));
  reconf_server_->setCallback(boost::bind(&TimeModification::configCB, this, _1, _2));

  seq_counter_ = 0;
  output_advertised_ = false;
}

void TimeModification::processShapeShifter(const topic_tools::ShapeShifter::Ptr shape_shifter)
{
    ROS_DEBUG("TimeModification processShapeShifter Ptr");
    processShapeShifter(*shape_shifter);
}

void TimeModification::processShapeShifter(topic_tools::ShapeShifter &shape_shifter)
{
    ROS_DEBUG("TimeModification processShapeShifter");
    std_msgs::Header header;
    uint8_t msg_buffer[shape_shifter.size()];
    ros::serialization::OStream o_stream(msg_buffer, shape_shifter.size());
    ros::serialization::IStream i_stream(msg_buffer, shape_shifter.size());
    shape_shifter.write(o_stream);
    header.seq = ((uint32_t *)msg_buffer)[0];
    header.stamp.sec = ((uint32_t *)msg_buffer)[1];
    header.stamp.nsec = ((uint32_t *)msg_buffer)[2];

    // TODO (fkunz) check if header data is consistent
    if ( fabs((header.stamp - ros::Time::now()).toSec()) > 5.0 )
    {
      ROS_WARN("Probably subscribed to a topic with a message type not "
               "containing a header.Timestamp is: %f at Time: %f",
               header.stamp.toSec(), ros::Time::now().toSec());
    }
    boost::mutex::scoped_lock config_lock(config_mutex_);
    ROS_DEBUG("setting header stamp from %f to %f", header.stamp.toSec(),
              (header.stamp + time_offset_).toSec());
    header.stamp += time_offset_;

    ((uint32_t *)msg_buffer)[0] = header.seq + seq_counter_++;
    ((uint32_t *)msg_buffer)[1] = header.stamp.sec;
    ((uint32_t *)msg_buffer)[2] = header.stamp.nsec;
    shape_shifter.read(i_stream);
}

void TimeModification::startPublisherThread(const ros::NodeHandle &nh)
{
    ROS_DEBUG("TimeModification startPublisherThread");
    boost::thread pub_thread(&TimeModification::publishMsgLoop, this, nh);
    pub_thread.detach();
}

void TimeModification::configCB(Config &config, uint32_t level)
{
  ROS_INFO("TimeModification configCB");
  msg_delay_milliseconds_ = config.msg_delay_milliseconds;
  time_offset_milliseconds_ = config.time_offset_milliseconds   ;
  boost::mutex::scoped_lock config_lock(config_mutex_);
  msg_delay_ = ros::Duration(msg_delay_milliseconds_/1000);
  time_offset_ = ros::Duration(time_offset_milliseconds_/1000);
  publish_retry_rate_ = ros::Rate(config.publish_retry_rate);
}

void TimeModification::inputCB(const topic_tools::ShapeShifter::ConstPtr &input)
{
  ROS_DEBUG("TimeModification inputCB");
  ros::Time start_time(ros::Time::now());
  if (!output_advertised_) {
    ROS_INFO("Output not advertised. Setting up publisher now.");
    ros::AdvertiseOptions opts("output", 1, input->getMD5Sum(),
                               input->getDataType(), input->getMessageDefinition());
    boost::mutex::scoped_lock pub_lock(pub_mutex_);
    generic_pub_ = private_nh_.advertise(opts);
    output_advertised_ = true;
  }
  uint8_t msg_buffer[input->size()];
  ros::serialization::OStream o_stream(msg_buffer, input->size());
  input->write(o_stream);
  processBuffer(msg_buffer);
//  topic_tools::ShapeShifter output;
  StampedMsg stamped_msg(msg_buffer, start_time + msg_delay_);
//  ros::serialization::IStream i_stream(msg_buffer, input->size());
//  stamped_msg.first.read(i_stream);
  boost::mutex::scoped_lock buffer_lock(buffer_mutex_);
  stamped_msg_buffer_.push(stamped_msg);
  ROS_DEBUG("all done");
}

void TimeModification::processBuffer(uint8_t *msg_buffer)
{
    ROS_DEBUG("TimeModification processShapeShifter");
    std_msgs::Header header;;
    header.seq = ((uint32_t *)msg_buffer)[0];
    header.stamp.sec = ((uint32_t *)msg_buffer)[1];
    header.stamp.nsec = ((uint32_t *)msg_buffer)[2];

    // TODO (fkunz) check if header data is consistent
    if ( fabs((header.stamp - ros::Time::now()).toSec()) > 5.0 )
    {
      ROS_WARN("Probably subscribed to a topic with a message type not "
               "containing a header.Timestamp is: %f at Time: %f",
               header.stamp.toSec(), ros::Time::now().toSec());
    }
    boost::mutex::scoped_lock config_lock(config_mutex_);
    ROS_DEBUG("setting header stamp from %f to %f", header.stamp.toSec(),
              (header.stamp + time_offset_).toSec());
    header.stamp += time_offset_;

    ((uint32_t *)msg_buffer)[0] = header.seq + seq_counter_++;
    ((uint32_t *)msg_buffer)[1] = header.stamp.sec;
    ((uint32_t *)msg_buffer)[2] = header.stamp.nsec;
}

void TimeModification::publishMsgLoop(const ros::NodeHandle &nh)
{
    ROS_DEBUG("TimeModification publishMsgLoop");
    while (nh.ok())
    {
        {
            boost::mutex::scoped_lock buffer_lock(buffer_mutex_);
            if (!stamped_msg_buffer_.empty())
            {
                ROS_DEBUG("publishing msg");
                publishMsg(stamped_msg_buffer_.front());
                stamped_msg_buffer_.pop();
                continue;
            }
        }
        publish_retry_rate_.sleep();
    }
}

void TimeModification::publishMsg(const StampedMsg &stamped_msg)
{
    ROS_DEBUG("TimeModification publishMsg");
    publishMsg(stamped_msg.first, stamped_msg.second);
}

void TimeModification::publishMsg(const uint8_t &msg_buffer, const ros::Time &time_to_pub)
    {
    ROS_DEBUG("TimeModification publishMsg Thread started with timestamp %f", time_to_pub.toSec());
    ros::Time end_time(ros::Time::now());
    ros::Time last_time;
    config_mutex_.lock();
    const ros::Duration initial_delay(msg_delay_);
    config_mutex_.unlock();
    do
    {
        last_time = end_time;
        ROS_DEBUG("waiting to publish msg at %f", time_to_pub.toSec());
        if (end_time > time_to_pub)
        {
            boost::mutex::scoped_lock pub_lock(pub_mutex_);
            ROS_DEBUG("publishing msg which should be held back till: %f", time_to_pub.toSec());
            topic_tools::ShapeShifter output;
            output.read(msg_buffer);
            generic_pub_.publish(output);
            return;
        }
        boost::mutex::scoped_lock config_lock(config_mutex_);
        publish_retry_rate_.sleep();
        end_time = ros::Time::now();
        if (initial_delay != msg_delay_)
        {
            ROS_WARN("Detected change of msg_delay. Dropping msg to be published at %f",
                     time_to_pub.toSec());
            return;
        }
    } while (last_time <= end_time);
    ROS_WARN("Detected jump back in time. Dropping msg. last: %f, end %f",
             last_time.toSec(), end_time.toSec());
    return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "time_modification_node");
  ros::NodeHandle nh;
  ros::Rate publish_rate(10);

  TimeModification time_modification_node(nh, publish_rate);
  ROS_INFO("Constructor successfull start spinning.");
  time_modification_node.startPublisherThread(nh);
  ROS_INFO("Publisher Thread started.");
  ros::spin();

  return 0;
}
