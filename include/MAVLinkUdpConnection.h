#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <map>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// MAVLink headers
#include <mavlink/v1.0/common/mavlink.h>
#include <mavlink/v2.0/common/mavlink.h>

// Forward declarations
class Vehicle;

/// MAVLink UDP connection handler for both v1 and v2 protocols.
/// This is a Qt-free implementation of QGroundControl's link system.
class MAVLinkUdpConnection
{
public:
    MAVLinkUdpConnection();
    virtual ~MAVLinkUdpConnection();

    /// Connect to a UDP endpoint
    /// @param targetAddress IP address to connect to
    /// @param targetPort Port to connect to
    /// @param localPort Local port to bind to (0 for automatic)
    /// @return true if connection successful
    bool connect(const std::string &targetAddress, uint16_t targetPort, uint16_t localPort = 0);

    /// Disconnect from the current endpoint
    void disconnect();

    /// Check if connected
    bool isConnected() const { return _connected; }

    /// Send a MAVLink message
    /// @param message MAVLink message to send
    /// @return true if sent successfully
    bool sendMessage(const mavlink_message_t &message);

    /// Send a MAVLink message to specific system/component
    /// @param message MAVLink message to send
    /// @param systemId Target system ID
    /// @param componentId Target component ID
    /// @return true if sent successfully
    bool sendMessage(const mavlink_message_t &message, uint8_t systemId, uint8_t componentId);

    /// Get local port
    uint16_t localPort() const { return _localPort; }

    /// Get remote address and port
    std::string remoteAddress() const { return _targetAddress; }
    uint16_t remotePort() const { return _targetPort; }

    /// Set vehicle for message handling
    void setVehicle(Vehicle *vehicle) { _vehicle = vehicle; }

    // Callback support for Qt-free implementation
    typedef std::function<void(const mavlink_message_t&)> MessageReceivedCallback;
    void setMessageReceivedCallback(MessageReceivedCallback callback) { _messageReceivedCallback = callback; }

    typedef std::function<void(bool)> ConnectionChangedCallback;
    void setConnectionChangedCallback(ConnectionChangedCallback callback) { _connectionChangedCallback = callback; }

    /// Get connection statistics
    uint64_t getBytesReceived() const { return _bytesReceived; }
    uint64_t getBytesSent() const { return _bytesSent; }
    uint64_t getPacketsReceived() const { return _packetsReceived; }
    uint64_t getPacketsSent() const { return _packetsSent; }
    uint64_t getPacketsLost() const { return _packetsLost; }

    /// Reset statistics
    void resetStatistics();

    /// Enable/disable automatic version detection
    void setAutoVersionDetection(bool enabled) { _autoVersionDetection = enabled; }
    bool autoVersionDetection() const { return _autoVersionDetection; }

    /// Get detected MAVLink version
    int getDetectedMavlinkVersion() const { return _detectedMavlinkVersion; }

private:
    void _receiveThreadFunc();
    void _processReceivedData(const uint8_t *data, size_t length, const std::string &senderAddress, uint16_t senderPort);
    bool _parseMavlinkData(const uint8_t *data, size_t length);
    void _detectMavlinkVersion(const mavlink_message_t &message);
    void _updateMessageLossStats(const mavlink_message_t &message);
    void _sendHeartbeatFunc();

    // Network socket
    int _socketFd;
    std::string _targetAddress;
    uint16_t _targetPort;
    uint16_t _localPort;
    std::atomic<bool> _connected{false};
    
    // Sender address tracking (for responses)
    struct sockaddr_in _lastSenderAddr;
    socklen_t _lastSenderAddrLen;
    std::atomic<bool> _haveSenderAddr{false};

    // Threading
    std::thread _receiveThread;
    std::atomic<bool> _running{false};
    std::mutex _socketMutex;

    // MAVLink processing
    mavlink_status_t _mavlinkStatus[MAVLINK_COMM_NUM_BUFFERS];
    std::atomic<int> _detectedMavlinkVersion{2}; // Default to v2
    bool _autoVersionDetection = true;

    // Vehicle reference
    Vehicle *_vehicle = nullptr;

    // Callbacks
    MessageReceivedCallback _messageReceivedCallback;
    ConnectionChangedCallback _connectionChangedCallback;

    // Statistics
    std::atomic<uint64_t> _bytesReceived{0};
    std::atomic<uint64_t> _bytesSent{0};
    std::atomic<uint64_t> _packetsReceived{0};
    std::atomic<uint64_t> _packetsSent{0};
    std::atomic<uint64_t> _packetsLost{0};
    
    // Message loss tracking
    std::set<std::pair<uint8_t, uint8_t>> _firstMessageSeen;
    std::map<std::pair<uint8_t, uint8_t>, uint8_t> _lastSequence;
    uint64_t _totalLossCounter = 0;
    double _runningLossPercent = 0.0f;

    // Heartbeat
    std::thread _heartbeatThread;
    std::atomic<bool> _sendHeartbeatFlag{false};
    static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000; // 1 second

    // System and component IDs
    uint8_t _systemId = 255;
    uint8_t _componentId = MAV_COMP_ID_PERIPHERAL;
};
