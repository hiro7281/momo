#ifndef RTC_MANAGER_H_
#define RTC_MANAGER_H_
#include <thread>
#include "api/peer_connection_interface.h"
#include "pc/video_track_source.h"

#include "observer.h"
#include "connection.h"
#include "connection_settings.h"
// #include "scalable_track_source.h"
#include "ros/ros_video_capture.h"

class DCO : public webrtc::DataChannelObserver {
 public:
  DCO(){};

  // 接続状況が変化した時に発火する。切断は発火タイミングで値を確認して検知可能
  void OnStateChange() override {
    std::cout << std::this_thread::get_id() << ":"
              << "DataChannelObserver::StateChange" << std::endl;
  };
  
  // メッセージ受信
  void OnMessage(const webrtc::DataBuffer& buffer) override {
    std::cout << std::this_thread::get_id() << ":"
              << "DataChannelObserver::Message" << std::endl;
    std::cout << std::string(buffer.data.data<char>(), buffer.data.size()) << std::endl;
  };

  void OnBufferedAmountChange(uint64_t previous_amount) override {
    std::cout << std::this_thread::get_id() << ":"
              << "DataChannelObserver::BufferedAmountChange(" << previous_amount << ")" << std::endl;
  };

  void Release(){};
};

class RTCManager
{
public:
  RTCManager(ConnectionSettings conn_settings,
             rtc::scoped_refptr<ROSVideoCapture> video_track_source);
  ~RTCManager();
  std::shared_ptr<RTCConnection> createConnection(
          webrtc::PeerConnectionInterface::RTCConfiguration rtc_config,
          RTCMessageSender *sender);
  // void ROSDataCallback(const std_msgs::String::ConstPtr& msg);
  // void ROSDataCallback(std_msgs::String msg);

  class PeerConnectionObserver : public webrtc::PeerConnectionObserver
  {
  public:
    RTCManager *rtc_manager;
    PeerConnectionObserver(RTCMessageSender *sender) : _sender(sender) {};

    void OnSignalingChange(
            webrtc::PeerConnectionInterface::SignalingState new_state) override {}
    void OnAddStream(
            rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) override {}
    void OnRemoveStream(
            rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) override {}
    void OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) override {
      std::cout << std::this_thread::get_id() << ":"
                << "PeerConnectionObserver::DataChannel(" << data_channel
                << ", " << rtc_manager->data_channel.get() << ")" << std::endl;
      // Answer送信側は、onDataChannelでDataChannelの接続を受け付ける
      //rtc_manager->data_channel = data_channel;
      std::cout << "aaa" << std::endl;
      //rtc_manager->data_channel->RegisterObserver(rtc_manager->dco);
      std::cout << "bbb" << std::endl;
    };
    void OnRenegotiationNeeded() override {}
    void OnIceConnectionChange(
                webrtc::PeerConnectionInterface::IceConnectionState new_state) override
    {
      _sender->onIceConnectionStateChange(new_state);
    }
    void OnIceGatheringChange(
            webrtc::PeerConnectionInterface::IceGatheringState new_state) override {};
    void OnIceCandidate(const webrtc::IceCandidateInterface *candidate) override
    {
      std::string sdp;
      if (candidate->ToString(&sdp))
      {
        if (_sender != nullptr)
        {
          _sender->onIceCandidate(candidate->sdp_mid(), candidate->sdp_mline_index(), sdp);
        }
      }
      else
      {
        RTC_LOG(LS_ERROR) << "Failed to serialize candidate";
      }
    }
    void OnIceConnectionReceivingChange(bool receiving) override {}
    void Release(){};

  private:
    RTCMessageSender *_sender;
  };
  ROSVideoCapture *vts;
  std::shared_ptr<RTCConnection> peer_connection;
  PeerConnectionObserver *pco;
  rtc::scoped_refptr<DCO> dco;
  rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel;

// private:
  rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> _factory;
  rtc::scoped_refptr<webrtc::VideoTrackSourceInterface> _video_source;
  std::unique_ptr<rtc::Thread> _networkThread;
  std::unique_ptr<rtc::Thread> _workerThread;
  std::unique_ptr<rtc::Thread> _signalingThread;
  ConnectionSettings _conn_settings;
};
#endif
