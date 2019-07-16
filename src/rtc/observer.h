#ifndef PEERCONNECTIONOBSERVER_H_
#define PEERCONNECTIONOBSERVER_H_

#include "api/peer_connection_interface.h"

#include "messagesender.h"

class CreateSessionDescriptionObserver : public webrtc::CreateSessionDescriptionObserver
{
public:
  static CreateSessionDescriptionObserver *Create(
          RTCMessageSender *sender, webrtc::PeerConnectionInterface *connection)
  {
    return new rtc::RefCountedObject<CreateSessionDescriptionObserver>(sender, connection);
  }

protected:
  CreateSessionDescriptionObserver(
          RTCMessageSender *sender, webrtc::PeerConnectionInterface *connection)
          : _sender(sender), _connection(connection) {};
  void OnSuccess(webrtc::SessionDescriptionInterface *desc) override;
  void OnFailure(webrtc::RTCError error) override;

private:
  RTCMessageSender *_sender;
  webrtc::PeerConnectionInterface *_connection;
};

class SetSessionDescriptionObserver
    : public webrtc::SetSessionDescriptionObserver
{
public:
  static SetSessionDescriptionObserver *Create(
          webrtc::SdpType type, RTCMessageSender *sender)
  {
    return new rtc::RefCountedObject<SetSessionDescriptionObserver>(type, sender);
  }

protected:
  SetSessionDescriptionObserver(webrtc::SdpType type, RTCMessageSender *sender)
          : _type(type), _sender(sender) {};
  void OnSuccess() override;
  void OnFailure(webrtc::RTCError error) override;

private:
  RTCMessageSender *_sender;
  webrtc::SdpType _type;
};
#endif