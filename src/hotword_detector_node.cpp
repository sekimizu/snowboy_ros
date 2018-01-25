#include "snowboy_ros/hotword_detector.h"

#include <ros/node_handle.h>
#include <ros/debug.h>
#include <audio_common_msgs/AudioData.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>
#include <snowboy_ros/SnowboyReconfigureConfig.h>

#include <boost/filesystem.hpp>

#include "portaudio/include/pa_ringbuffer.h"
#include "portaudio/include/pa_util.h"
#include "portaudio/include/portaudio.h"

int PortAudioCallback(const void* input,
                      void* output,
                      unsigned long frame_count,
                      const PaStreamCallbackTimeInfo* time_info,
                      PaStreamCallbackFlags status_flags,
                      void* user_data);


namespace snowboy_ros
{

//!
//! \brief The HotwordDetectorNode class Wraps the C++ 11 Snowboy detector in a ROS node
//!
class HotwordDetectorNode
{
public:
  HotwordDetectorNode():
    nh_(""),
    nh_p_("~")
  {
  }

  //!
  //! \brief detector_ C++ 11 Wrapped Snowboy detect
  //!
  HotwordDetector detector_;

  bool initialize()
  {
    //hotword_pub_ = nh_.advertise<std_msgs::String>("hotword_detection", 10);

    std::string resource_filename;
    if (!nh_p_.getParam("resource_filename", resource_filename))
    {
      ROS_ERROR("Mandatory parameter 'resource_filename' not present on the parameter server");
      return false;
    }

    if ( !boost::filesystem::exists( resource_filename ) )
    {
      ROS_ERROR("Resource '%s' does not exist", resource_filename.c_str());
      return false;
    }

    std::string resource_extension = boost::filesystem::extension(resource_filename);
    if ( resource_extension != ".res" )
    {
      ROS_ERROR("'%s' not a valid Snowboy resource extension ('.res').", resource_filename.c_str());
      return false;
    }

    std::string model_filename;
    if (!nh_p_.getParam("model_filename", model_filename))
    {
      ROS_ERROR("Mandatory parameter 'model_filename' not present on the parameter server");
      return false;
    }

    if ( !boost::filesystem::exists( model_filename ) )
    {
      ROS_ERROR("Model '%s' does not exist", model_filename.c_str());
      return false;
    }

    std::string model_extension = boost::filesystem::extension(model_filename);
    if ( model_extension  != ".pmdl" && model_extension != ".umdl" )
    {
      ROS_ERROR("Model '%s', not a valid Snowboy model extension ('.pmdl', '.umdl').", resource_filename.c_str());
      return false;
    }

    detector_.initialize(resource_filename.c_str(), model_filename.c_str());

    dynamic_reconfigure_server_.setCallback(boost::bind(&HotwordDetectorNode::reconfigureCallback, this, _1, _2));

    return true;
  }

private:

  //!
  //! \brief nh_ Global nodehandle for topics
  //!
  ros::NodeHandle nh_;

  //!
  //! \brief nh_p_ Local nodehandle for parameters
  //!
  ros::NodeHandle nh_p_;

  //!
  //! \brief audio_sub_ Subscriber to incoming audio feed
  //!
  ros::Subscriber audio_sub_;

  //!
  //! \brief hotword_pub_ hotword publisher
  //!
  ros::Publisher hotword_pub_;

  //!
  //! \brief dynamic_reconfigure_server_ In order to online tune the sensitivity and audio gain
  //!
  dynamic_reconfigure::Server<SnowboyReconfigureConfig> dynamic_reconfigure_server_;

  //!
  //! \brief reconfigureCallback Reconfigure update for sensitiviy and audio level
  //! \param cfg The updated config
  //!
  void reconfigureCallback(SnowboyReconfigureConfig cfg, uint32_t /*level*/)
  {
    detector_.configure(cfg.sensitivity, cfg.audio_gain);
    ROS_INFO("SnowboyROS (Re)Configured");
  }

};

}

class PortAudioWrapper {
 public:
  // Constructor.
  PortAudioWrapper(int sample_rate, int num_channels, int bits_per_sample) {
    num_lost_samples_ = 0;
    min_read_samples_ = sample_rate * 0.1;
    Init(sample_rate, num_channels, bits_per_sample);
  }

  // Reads data from ring buffer.
  template<typename T>
  void Read(std::vector<T>* data) {
    assert(data != NULL);

    // Checks ring buffer overflow.
    if (num_lost_samples_ > 0) {
      std::cerr << "Lost " << num_lost_samples_ << " samples due to ring"
          << " buffer overflow." << std::endl;
      num_lost_samples_ = 0;
    }

    ring_buffer_size_t num_available_samples = 0;
    while (true) {
      num_available_samples =
          PaUtil_GetRingBufferReadAvailable(&pa_ringbuffer_);
      if (num_available_samples >= min_read_samples_) {
        break;
      }
      Pa_Sleep(5);
    }

    // Reads data.
    num_available_samples = PaUtil_GetRingBufferReadAvailable(&pa_ringbuffer_);
    data->resize(num_available_samples);
    ring_buffer_size_t num_read_samples = PaUtil_ReadRingBuffer(
        &pa_ringbuffer_, data->data(), num_available_samples);
    if (num_read_samples != num_available_samples) {
      std::cerr << num_available_samples << " samples were available,  but "
          << "only " << num_read_samples << " samples were read." << std::endl;
    }
  }

  int Callback(const void* input, void* output,
               unsigned long frame_count,
               const PaStreamCallbackTimeInfo* time_info,
               PaStreamCallbackFlags status_flags) {
    // Input audio.
    ring_buffer_size_t num_written_samples =
        PaUtil_WriteRingBuffer(&pa_ringbuffer_, input, frame_count);
    num_lost_samples_ += frame_count - num_written_samples;
    return paContinue;
  }

  ~PortAudioWrapper() {
    Pa_StopStream(pa_stream_);
    Pa_CloseStream(pa_stream_);
    Pa_Terminate();
    PaUtil_FreeMemory(ringbuffer_);
  }

 private:
  // Initialization.
  bool Init(int sample_rate, int num_channels, int bits_per_sample) {
    // Allocates ring buffer memory.
    int ringbuffer_size = 16384;
    ringbuffer_ = static_cast<char*>(
        PaUtil_AllocateMemory(bits_per_sample / 8 * ringbuffer_size));
    if (ringbuffer_ == NULL) {
      std::cerr << "Fail to allocate memory for ring buffer." << std::endl;
      return false;
    }

    // Initializes PortAudio ring buffer.
    ring_buffer_size_t rb_init_ans =
        PaUtil_InitializeRingBuffer(&pa_ringbuffer_, bits_per_sample / 8,
                                    ringbuffer_size, ringbuffer_);
    if (rb_init_ans == -1) {
      std::cerr << "Ring buffer size is not power of 2." << std::endl;
      return false;
    }

    // Initializes PortAudio.
    PaError pa_init_ans = Pa_Initialize();
    if (pa_init_ans != paNoError) {
      std::cerr << "Fail to initialize PortAudio, error message is \""
          << Pa_GetErrorText(pa_init_ans) << "\"" << std::endl;
      return false;
    }

    PaError pa_open_ans;
    if (bits_per_sample == 8) {
      pa_open_ans = Pa_OpenDefaultStream(
          &pa_stream_, num_channels, 0, paUInt8, sample_rate,
          paFramesPerBufferUnspecified, PortAudioCallback, this);
    } else if (bits_per_sample == 16) {
      pa_open_ans = Pa_OpenDefaultStream(
          &pa_stream_, num_channels, 0, paInt16, sample_rate,
          paFramesPerBufferUnspecified, PortAudioCallback, this);
    } else if (bits_per_sample == 32) {
      pa_open_ans = Pa_OpenDefaultStream(
          &pa_stream_, num_channels, 0, paInt32, sample_rate,
          paFramesPerBufferUnspecified, PortAudioCallback, this);
    } else {
      std::cerr << "Unsupported BitsPerSample: " << bits_per_sample
          << std::endl;
      return false;
    }
    if (pa_open_ans != paNoError) {
      std::cerr << "Fail to open PortAudio stream, error message is \""
          << Pa_GetErrorText(pa_open_ans) << "\"" << std::endl;
      return false;
    }

    PaError pa_stream_start_ans = Pa_StartStream(pa_stream_);
    if (pa_stream_start_ans != paNoError) {
      std::cerr << "Fail to start PortAudio stream, error message is \""
          << Pa_GetErrorText(pa_stream_start_ans) << "\"" << std::endl;
      return false;
    }
    return true;
  }

 private:
  // Pointer to the ring buffer memory.
  char* ringbuffer_;

  // Ring buffer wrapper used in PortAudio.
  PaUtilRingBuffer pa_ringbuffer_;

  // Pointer to PortAudio stream.
  PaStream* pa_stream_;

  // Number of lost samples at each Read() due to ring buffer overflow.
  int num_lost_samples_;

  // Wait for this number of samples in each Read() call.
  int min_read_samples_;
};

int PortAudioCallback(const void* input,
                      void* output,
                      unsigned long frame_count,
                      const PaStreamCallbackTimeInfo* time_info,
                      PaStreamCallbackFlags status_flags,
                      void* user_data) {
  PortAudioWrapper* pa_wrapper = reinterpret_cast<PortAudioWrapper*>(user_data);
  pa_wrapper->Callback(input, output, frame_count, time_info, status_flags);
  return paContinue;
}

int main(int argc, char** argv)
{
  ROS_DEBUG("Launch hotword_detector_node...\n");
  ros::init(argc, argv, "snowboy_node");
  ros::Publisher publisher;
  ros::NodeHandle nh;

  snowboy_ros::HotwordDetectorNode ros_hotword_detector_node;

  if (ros_hotword_detector_node.initialize())
  {
    ROS_DEBUG("Listening...\n");
    std::vector<int16_t> data;

    publisher = nh.advertise<std_msgs::String>("hotword_detection", 10);
    ros::Rate timer(100);

    PortAudioWrapper pa_wrapper(ros_hotword_detector_node.detector_.samplerate_, ros_hotword_detector_node.detector_.num_channels_, ros_hotword_detector_node.detector_.bits_per_sample_);

    while(ros::ok()) {
      pa_wrapper.Read(&data);
      if (data.size() != 0) {
        int result = ros_hotword_detector_node.detector_.runDetection(data.data(), data.size());
        if (result > 0) {
          ROS_INFO("Hotword detected!");
          std_msgs::String hotword_msg;
          hotword_msg.data = std::string("hiaqua");
          publisher.publish(hotword_msg);
        }
      }
      timer.sleep();
    }

//    ros::spin();
  }
  else
  {
    ROS_ERROR("Failed to initialize snowboy_node");
    return 1;
  }

  return 0;
}

