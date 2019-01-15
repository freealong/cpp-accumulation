//
// Created by yongqi on 17-11-14.
//

#ifndef VFORCE_SYNCTCPSERVER_HPP
#define VFORCE_SYNCTCPSERVER_HPP

#include <sstream>
#include <iomanip>
#include <boost/asio.hpp>

namespace utcp {

class Message {
 public:
  /**
   * we store the whole message's size in 10 length string,
   * message's type in 3 length string.
   * For example, if the message string is "000000017000fuck",
   * then the whole message's size is "000000017", type is "000",
   * data is "fuck".
   * type == "000" means string
   * type == "001" means image
   * type == "002" means compressed image
   * type == "003" means controller cfg
   */
  const static unsigned length_of_size = 10;
  const static unsigned length_of_type = 3;
  const static unsigned length_of_header = length_of_type + length_of_size;

  std::string data;
  std::string header;
  Message() : header(length_of_header, '0') {}

  std::string get_type() const {
    return header.substr(length_of_size, length_of_type);
  }

  void PackString(const std::string & s) {
    auto data_size = s.size();
    std::stringstream ss;
    ss << std::setw(length_of_size) << std::setfill('0') << data_size;
    header = ss.str() + "000";
    data = s;
  }

  bool UnpackString(std::string &s) {
    if (get_type() != "000") {
      return false;
    } else {
      s = data;
      return true;
    }
  }
};

class SyncTCP {
 public:
  static boost::shared_ptr<boost::asio::ip::tcp::acceptor> CreateAcceptor(boost::asio::io_service &io_service,
      const std::string &address, unsigned short port) {
    auto addr = boost::asio::ip::address::from_string(address);
    return boost::shared_ptr<boost::asio::ip::tcp::acceptor>(
        new boost::asio::ip::tcp::acceptor(io_service, boost::asio::ip::tcp::endpoint(addr, port)));
  }

  static boost::shared_ptr<boost::asio::ip::tcp::resolver> CreateResolver(boost::asio::io_service &io_service) {
    return boost::shared_ptr<boost::asio::ip::tcp::resolver>(
        new boost::asio::ip::tcp::resolver(io_service));
  }

  /**
   * Receive message
   * @param msg the received message
   * @return read bytes size
   */
  template <typename MsgT>
  void RecvMsg(MsgT &msg) {
    boost::asio::read(*socket_, boost::asio::buffer(&(msg.header[0]), MsgT::length_of_header));
    auto data_size = static_cast<size_t >(std::stoi(msg.header.substr(0, MsgT::length_of_size)));
    if (data_size > 0) {
      msg.data_.resize(data_size);
      auto read_size = boost::asio::read(*socket_, boost::asio::buffer(&(msg.data[0]), data_size));
    }
  }

  /**
   * Send message to client
   * @param msg the message will be sent
   * @return write bytes size
   */
  template <typename MsgT>
  void SendMsg(const MsgT &msg) {
    boost::asio::write(*socket_, boost::asio::buffer(&(msg.header[0]), MsgT::length_of_header));
    if (not msg.data_.empty()) {
      auto send_size = boost::asio::write(*socket_, boost::asio::buffer(&(msg.data[0]), msg.data.size()));
    }
  }

  /**
   * Receive raw message, socket will read raw.size() bytes
   * @param raw
   */
  void RecvRaw(std::string &raw) {
    boost::asio::read(*socket_, boost::asio::buffer(&(raw[0]), raw.size()));
  }

  /**
   * Send raw message
   * @param raw
   */
  void SendRaw(const std::string &raw) {
    boost::asio::write(*socket_, boost::asio::buffer(&(raw[0]), raw.size()));
  }

 protected:
  boost::shared_ptr<boost::asio::ip::tcp::socket> socket_;
};

class SyncTCPServer : public SyncTCP {
 public:
  explicit SyncTCPServer(boost::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor) : acceptor_(std::move(acceptor)) {
    this->socket_ = boost::shared_ptr<boost::asio::ip::tcp::socket>(
        new boost::asio::ip::tcp::socket(acceptor_->get_io_service()));
  }

  void WaitingClient() {
    acceptor_->accept(*(this->socket_));
  }

 private:
  boost::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor_;
};

class SyncTCPClient : public SyncTCP {
 public:
  explicit SyncTCPClient(boost::shared_ptr<boost::asio::ip::tcp::resolver> resolver) : resolver_(std::move(resolver)) {
    this->socket_ = boost::shared_ptr<boost::asio::ip::tcp::socket>(
        new boost::asio::ip::tcp::socket(resolver_->get_io_service()));
  }

  void Connect(const std::string &address, unsigned short port) {
    boost::asio::ip::tcp::resolver::query query(address, std::to_string(port));
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver_->resolve(query);
    boost::asio::connect(*(this->socket_), endpoint_iterator);
  }

 private:
  boost::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
};

}
#endif //VFORCE_SYNCTCPSERVER_HPP
