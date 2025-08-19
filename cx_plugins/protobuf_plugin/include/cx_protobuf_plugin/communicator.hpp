// Copyright (c) 2024-2025 Carologistics
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Library General Public License for more details.
//
// Read the full text in the LICENSE.GPL file in the main directory.

/***************************************************************************
 *  communicator.h - protobuf network communication for CLIPS
 *
 *  Created: Tue Apr 16 13:41:13 2013
 *  Copyright  2013-2014  Tim Niemueller [www.niemueller.de]
 *             2021       Till Hofmann <hofmann@kbsg.rwth-aachen.de>
 *             2924       Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
 ****************************************************************************/

#ifndef _PROTOBUF_CLIPS_COMMUNICATOR_H_
#define _PROTOBUF_CLIPS_COMMUNICATOR_H_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <protobuf_comm/server.h>

#include <clips_ns/clips.h>
#include <list>
#include <map>
#include <mutex>

namespace protobuf_comm {
class ProtobufStreamClient;
class ProtobufBroadcastPeer;
} // namespace protobuf_comm

namespace fawkes {
class Logger;
}

namespace protobuf_clips {

class ClipsProtobufCommunicator {
public:
  ClipsProtobufCommunicator(clips::Environment *env, std::mutex &env_mutex,
                            rclcpp_lifecycle::LifecycleNode::WeakPtr parent);
  ClipsProtobufCommunicator(clips::Environment *env, std::mutex &env_mutex,
                            std::vector<std::string> &proto_path,
                            rclcpp_lifecycle::LifecycleNode::WeakPtr parent);
  ~ClipsProtobufCommunicator();

  void enable_server(int port);
  void disable_server();

  /** Signal invoked for a message that has been sent to a server client.
   * @return signal
   */
  boost::signals2::signal<void(protobuf_comm::ProtobufStreamServer::ClientID,
                               std::shared_ptr<google::protobuf::Message>)> &
  signal_server_sent() {
    return sig_server_sent_;
  }

  /** Signal invoked for a message that has been sent to a client.
   * @return signal
   */
  boost::signals2::signal<void(std::string, unsigned short,
                               std::shared_ptr<google::protobuf::Message>)> &
  signal_client_sent() {
    return sig_client_sent_;
  }

  /** Signal invoked for a message that has been sent via broadcast.
   * @return signal
   */
  boost::signals2::signal<void(long,
                               std::shared_ptr<google::protobuf::Message>)> &
  signal_peer_sent() {
    return sig_peer_sent_;
  }

private:
  void setup_clips();

  clips::UDFValue clips_pb_register_type(std::string full_name);
  clips::UDFValue clips_pb_field_names(void *msgptr);
  clips::UDFValue clips_pb_has_field(void *msgptr, std::string field_name);
  clips::UDFValue clips_pb_field_value(void *msgptr, std::string field_name);
  clips::UDFValue clips_pb_field_type(void *msgptr, std::string field_name);
  clips::UDFValue clips_pb_field_label(void *msgptr, std::string field_name);
  clips::UDFValue clips_pb_field_list(void *msgptr, std::string field_name);
  clips::UDFValue clips_pb_field_is_list(void *msgptr, std::string field_name);
  clips::UDFValue clips_pb_create(std::string full_name);
  void clips_pb_destroy(void *msgptr);
  void clips_pb_set_field(void *msgptr, std::string field_name,
                          clips::UDFValue value);
  void clips_pb_add_list(void *msgptr, std::string field_name,
                         clips::UDFValue value);
  void clips_pb_send(long int client_id, void *msgptr);
  std::string clips_pb_tostring(void *msgptr);
  long int clips_pb_client_connect(std::string host, int port);
  void clips_pb_disconnect(long int client_id);
  void clips_pb_broadcast(long int peer_id, void *msgptr);

  long int clips_pb_peer_create(std::string host, int port);
  long int clips_pb_peer_create_local(std::string host, int send_port,
                                      int recv_port);
  long int clips_pb_peer_create_crypto(std::string host, int port,
                                       std::string crypto_key = "",
                                       std::string cipher = "");
  long int clips_pb_peer_create_local_crypto(std::string host, int send_port,
                                             int recv_port,
                                             std::string crypto_key = "",
                                             std::string cipher = "");
  void clips_pb_peer_destroy(long int peer_id);
  void clips_pb_peer_setup_crypto(long int peer_id, std::string crypto_key,
                                  std::string cipher);

  typedef enum { CT_SERVER, CT_CLIENT, CT_PEER } ClientType;
  void clips_assert_message(std::pair<std::string, unsigned short> &endpoint,
                            uint16_t comp_id, uint16_t msg_type,
                            std::shared_ptr<google::protobuf::Message> &msg,
                            ClientType ct, long int client_id = 0);
  void handle_server_client_connected(
      protobuf_comm::ProtobufStreamServer::ClientID client,
      boost::asio::ip::tcp::endpoint &endpoint);
  void handle_server_client_disconnected(
      protobuf_comm::ProtobufStreamServer::ClientID client,
      const boost::system::error_code &error);

  void
  handle_server_client_msg(protobuf_comm::ProtobufStreamServer::ClientID client,
                           uint16_t component_id, uint16_t msg_type,
                           std::shared_ptr<google::protobuf::Message> msg);

  void handle_server_client_fail(
      protobuf_comm::ProtobufStreamServer::ClientID client,
      uint16_t component_id, uint16_t msg_type, std::string msg);

  void handle_peer_msg(long int peer_id,
                       boost::asio::ip::udp::endpoint &endpoint,
                       uint16_t component_id, uint16_t msg_type,
                       std::shared_ptr<google::protobuf::Message> msg);
  void handle_peer_recv_error(long int peer_id,
                              boost::asio::ip::udp::endpoint &endpoint,
                              std::string msg);
  void handle_peer_send_error(long int peer_id, std::string msg);

  void handle_client_connected(long int client_id);
  void handle_client_disconnected(long int client_id,
                                  const boost::system::error_code &error);
  void handle_client_msg(long int client_id, uint16_t comp_id,
                         uint16_t msg_type,
                         std::shared_ptr<google::protobuf::Message> msg);
  void handle_client_receive_fail(long int client_id, uint16_t comp_id,
                                  uint16_t msg_type, std::string msg);

  static std::string to_string(const clips::UDFValue &v);

private:
  clips::Environment *clips_;
  std::mutex &clips_mutex_;

  std::unique_ptr<protobuf_comm::MessageRegister> message_register_;
  std::unique_ptr<protobuf_comm::ProtobufStreamServer> server_;

  boost::signals2::signal<void(protobuf_comm::ProtobufStreamServer::ClientID,
                               std::shared_ptr<google::protobuf::Message>)>
      sig_server_sent_;
  boost::signals2::signal<void(std::string, unsigned short,
                               std::shared_ptr<google::protobuf::Message>)>
      sig_client_sent_;
  boost::signals2::signal<void(long int,
                               std::shared_ptr<google::protobuf::Message>)>
      sig_peer_sent_;

  std::mutex map_mutex_;
  long int next_client_id_;

  std::map<long int, protobuf_comm::ProtobufStreamServer::ClientID>
      server_clients_;
  typedef std::map<protobuf_comm::ProtobufStreamServer::ClientID, long int>
      RevServerClientMap;
  RevServerClientMap rev_server_clients_;
  std::map<long int, std::unique_ptr<protobuf_comm::ProtobufStreamClient>>
      clients_;
  std::map<long int, std::unique_ptr<protobuf_comm::ProtobufBroadcastPeer>>
      peers_;

  std::map<long int, std::pair<std::string, unsigned short>> client_endpoints_;

  std::list<std::string> functions_;
  /// Reference to parent node to get ros time
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

  std::unordered_map<void *, std::shared_ptr<google::protobuf::Message>>
      messages_;

  std::unique_ptr<rclcpp::Logger> logger_;
};

} // end namespace protobuf_clips

#endif
