#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/event_groups.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "nvs.h"
#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <new>
#include <machine/endian.h>
#include <string.h>

#include <sys/param.h>

#include "lwip/sockets.h"
#include "portmacro.h"
#include "protocol/udp.hpp"
#include <lwip/netdb.h>

#include "network.hpp"

namespace {
constexpr std::size_t wifiSsidCapacity = 33;
constexpr std::size_t wifiPasswordCapacity = 65;

bool loadWifiCredentials(char *ssid, std::size_t ssidCapacity,
                         char *password, std::size_t passwordCapacity) {
  nvs_handle_t handle{};
  if (nvs_open("rover_net", NVS_READONLY, &handle) != ESP_OK) return false;
  std::size_t ssidLength = ssidCapacity;
  std::size_t passwordLength = passwordCapacity;
  const esp_err_t ssidResult = nvs_get_str(handle, "ssid", ssid, &ssidLength);
  const esp_err_t passwordResult =
      nvs_get_str(handle, "password", password, &passwordLength);
  nvs_close(handle);
  return ssidResult == ESP_OK && passwordResult == ESP_OK &&
         ssidLength > 1 && ssidLength <= ssidCapacity &&
         passwordLength <= passwordCapacity;
}

bool provisionWifiCredentials() {
  if (!usb_serial_jtag_is_driver_installed()) {
    usb_serial_jtag_driver_config_t driverConfig{};
    driverConfig.tx_buffer_size = 256;
    driverConfig.rx_buffer_size = 256;
    if (usb_serial_jtag_driver_install(&driverConfig) != ESP_OK) {
      ESP_LOGE(WIFI_TAG, "USB Serial/JTAG provisioning driver unavailable");
      return false;
    }
  }

  constexpr char prompt[] =
      "\r\nWIFI SETUP: enter WIFI|<ssid>|<password> then press Enter.\r\n"
      "The password is stored in NVS and is not echoed.\r\n";
  usb_serial_jtag_write_bytes(prompt, sizeof(prompt) - 1, pdMS_TO_TICKS(1000));

  char line[160]{};
  std::size_t lineLength = 0;
  bool skipNextLineFeed = false;
  while (true) {
    char value = 0;
    const int received = usb_serial_jtag_read_bytes(
        &value, 1, pdMS_TO_TICKS(250));
    if (received <= 0) continue;
    if (value == '\r') {
      value = '\n';
    } else if (value == '\n' && skipNextLineFeed) {
      skipNextLineFeed = false;
      continue;
    } else {
      skipNextLineFeed = false;
    }
    if (value != '\n') {
      if (lineLength + 1 < sizeof(line)) {
        line[lineLength++] = value;
      } else {
        lineLength = 0;
        constexpr char overflow[] = "Input too long. Try again.\r\n";
        usb_serial_jtag_write_bytes(overflow, sizeof(overflow) - 1,
                                    pdMS_TO_TICKS(100));
      }
      continue;
    }

    skipNextLineFeed = true;

    line[lineLength] = '\0';
    char *separator = std::strchr(line, '|');
    char *passwordSeparator = separator != nullptr
        ? std::strchr(separator + 1, '|') : nullptr;
    if (std::strncmp(line, "WIFI|", 5) != 0 || separator != line + 4 ||
        passwordSeparator == nullptr) {
      constexpr char invalid[] = "Expected WIFI|<ssid>|<password>. Try again.\r\n";
      usb_serial_jtag_write_bytes(invalid, sizeof(invalid) - 1,
                                  pdMS_TO_TICKS(100));
      lineLength = 0;
      continue;
    }

    *passwordSeparator = '\0';
    const char *ssid = separator + 1;
    const char *password = passwordSeparator + 1;
    const std::size_t ssidLength = std::strlen(ssid);
    const std::size_t passwordLength = std::strlen(password);
    if (ssidLength == 0 || ssidLength > 32 || passwordLength > 63) {
      constexpr char invalid[] = "SSID/password length is invalid. Try again.\r\n";
      usb_serial_jtag_write_bytes(invalid, sizeof(invalid) - 1,
                                  pdMS_TO_TICKS(100));
      lineLength = 0;
      continue;
    }

    nvs_handle_t handle{};
    esp_err_t result = nvs_open("rover_net", NVS_READWRITE, &handle);
    const bool handleOpened = result == ESP_OK;
    if (result == ESP_OK) result = nvs_set_str(handle, "ssid", ssid);
    if (result == ESP_OK) result = nvs_set_str(handle, "password", password);
    if (result == ESP_OK) result = nvs_commit(handle);
    if (handleOpened) nvs_close(handle);
    if (result != ESP_OK) {
      ESP_LOGE(WIFI_TAG, "Could not save Wi-Fi credentials to NVS: %s",
               esp_err_to_name(result));
      lineLength = 0;
      continue;
    }
    constexpr char saved[] = "Wi-Fi credentials saved to NVS.\r\n";
    usb_serial_jtag_write_bytes(saved, sizeof(saved) - 1,
                                pdMS_TO_TICKS(100));
    return true;
  }
}

bool getWifiCredentials(wifi_config_t &wifiConfig) {
  char ssid[wifiSsidCapacity]{};
  char password[wifiPasswordCapacity]{};
  if (!loadWifiCredentials(ssid, sizeof(ssid), password, sizeof(password))) {
    ESP_LOGW(WIFI_TAG, "No stored Wi-Fi credentials; waiting for USB provisioning");
    if (!provisionWifiCredentials() ||
        !loadWifiCredentials(ssid, sizeof(ssid), password, sizeof(password))) {
      return false;
    }
  }
  std::memcpy(wifiConfig.sta.ssid, ssid, std::strlen(ssid));
  std::memcpy(wifiConfig.sta.password, password, std::strlen(password));
  wifiConfig.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  return true;
}
} // namespace

bool UDPServer::initWIFI() {
  ESP_LOGI(WIFI_TAG, "Connecting to WiFi in station mode");
  wifiEventGroup = xEventGroupCreate();
  if (wifiEventGroup == nullptr) {
    ESP_LOGE(WIFI_TAG, "Could not allocate Wi-Fi event group");
    return false;
  }

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  if (esp_netif_create_default_wifi_sta() == nullptr) {
    ESP_LOGE(WIFI_TAG, "Could not create Wi-Fi station network interface");
    return false;
  }

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &eventHandler, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &eventHandler, NULL, &instance_got_ip));

  wifi_config_t wifi_config{};
  if (!getWifiCredentials(wifi_config)) return false;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(WIFI_TAG, "WiFi initialization finished.");

  while (true) {
    EventBits_t bits =
        xEventGroupWaitBits(wifiEventGroup, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                            pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
      ESP_LOGI(WIFI_TAG, "Connected to configured AP");
      return true;
    }
    if ((bits & WIFI_FAIL_BIT) == 0) {
      ESP_LOGE(WIFI_TAG, "Unexpected Wi-Fi event state");
      return false;
    }

    ESP_LOGW(WIFI_TAG, "Wi-Fi connection failed; update credentials over USB");
    if (!provisionWifiCredentials() || !getWifiCredentials(wifi_config)) {
      return false;
    }
    xEventGroupClearBits(wifiEventGroup, WIFI_FAIL_BIT | WIFI_CONNECTED_BIT);
    connectRetries = 0;
    if (esp_wifi_set_config(WIFI_IF_STA, &wifi_config) != ESP_OK) return false;
    esp_wifi_connect();
  }
}

int UDPServer::createSocket() {
  struct sockaddr_in dest_addr = {};
  int ip_protocol = IPPROTO_IP;

  dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(PORT);

  int sock = socket(AF_INET, SOCK_DGRAM, ip_protocol);
  if (sock < 0) {
    ESP_LOGE(WIFI_TAG, "Unable to create socket...error %d", errno);
    return -1;
  }
  ESP_LOGI(WIFI_TAG, "Socket created");

  int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (err < 0) {
    ESP_LOGE(WIFI_TAG, "Unable to bind socket...error %d)", errno);
    close(sock);
    return -1;
  }
  ESP_LOGI(WIFI_TAG, "Socket bound on port %d", PORT);

  port = PORT;
  char localIp[IP_LEN]{};
  unsigned short localPort = 0;
  if (getLocalAddress(localIp, sizeof(localIp), localPort)) {
    ESP_LOGI(WIFI_TAG, "UDP server address %s:%u", localIp,
             static_cast<unsigned>(localPort));
  }
  return sock;
}

bool UDPServer::getLocalAddress(char *localIp, std::size_t ipCapacity,
                                unsigned short &localPort) {
  if (localIp == nullptr || ipCapacity == 0) return false;
  localIp[0] = '\0';
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (!netif) {
    ESP_LOGE(WIFI_TAG, "Failed to get netif to read IP");
    return false;
  }

  esp_netif_ip_info_t ipInfo{};
  if (esp_netif_get_ip_info(netif, &ipInfo) != ESP_OK ||
      ipInfo.ip.addr == 0) {
    return false;
  }

  char address[IP_LEN]{};
  snprintf(address, sizeof(address), IPSTR, IP2STR(&ipInfo.ip));
  if (address[0] == '\0') return false;
  snprintf(localIp, ipCapacity, "%s", address);
  localPort = port;
  return true;
}

void UDPServer::loopListen() {
  char recvBuffer[BUFFER_LEN + 1];
  sockaddr_in clientAddr{};
  MOVE_CMD lastLoggedMove{};
  bool hasLoggedMove = false;
  std::uint32_t lastAuthenticatedSequence = 0;
  bool hasAuthenticatedSequence = false;
  TickType_t lastReceiveErrorLogTick = 0;
  int lastLoggedReceiveError = 0;
  bool hasLoggedReceiveError = false;

  while (true) {
    socklen_t clientAddrSize = sizeof(clientAddr);
    int bytesReceived =
        recvfrom(sock, (char *)recvBuffer, sizeof(recvBuffer), 0,
                 (struct sockaddr *)&clientAddr, &clientAddrSize);

    if (bytesReceived < 0) {
      const int receiveError = errno;
      if (receiveError == EAGAIN || receiveError == EWOULDBLOCK ||
          receiveError == EINTR) {
        // A timeout/nonblocking socket must yield even when no packet arrives.
        vTaskDelay(1);
        continue;
      }

      // Stop through the motor task's existing queues. A failed recvfrom can
      // return immediately on every retry; without backoff this high-priority
      // network task could starve the IMU and thermal tasks on the same core.
      disarmDrive();
      (void)handleMoveCommand(MOVE_CMD{});
      (void)handleMotionStopCommand();
      const TickType_t now = xTaskGetTickCount();
      if (!hasLoggedReceiveError || receiveError != lastLoggedReceiveError ||
          static_cast<TickType_t>(now - lastReceiveErrorLogTick) >=
              pdMS_TO_TICKS(5000)) {
        ESP_LOGE(WIFI_TAG, "UDP receive failed: errno %d; drive disarmed, retrying",
                 receiveError);
        lastLoggedReceiveError = receiveError;
        lastReceiveErrorLogTick = now;
        hasLoggedReceiveError = true;
      }
      constexpr TickType_t retryDelay =
          pdMS_TO_TICKS(100) > 0 ? pdMS_TO_TICKS(100) : 1;
      vTaskDelay(retryDelay);
      continue;
    } else if (bytesReceived >= sizeof(PacketHeader) &&
               bytesReceived <= BUFFER_LEN) {
      PacketHeader header{};
      memcpy(&header, recvBuffer, sizeof(header));
      if (header.version != PROTOCOL_VERSION ||
          header.length != bytesReceived - sizeof(PacketHeader)) {
        ESP_LOGW(WIFI_TAG, "Dropping malformed packet (%d bytes)", bytesReceived);
        continue;
      }

      char senderIp[INET_ADDRSTRLEN]{};
      if (inet_ntop(AF_INET, &clientAddr.sin_addr, senderIp,
                    sizeof(senderIp)) == nullptr) {
        continue;
      }
      const unsigned short senderPort = ntohs(clientAddr.sin_port);

      if (header.type == COMMAND && header.id == PAIR) {
        PAIR_REQUEST request{};
        const bool validRequest = header.authToken == 0 &&
            header.length == sizeof(PAIR_REQUEST);
        if (validRequest) {
          memcpy(&request, recvBuffer + sizeof(PacketHeader), sizeof(request));
        }
        const bool accepted = validRequest &&
            connectPeer(request.sessionToken, senderIp, senderPort);
        if (accepted) {
          hasAuthenticatedSequence = false;
          disarmDrive();
          MOVE_CMD stopCommand{};
          handleMoveCommand(stopCommand);
        }
        COMMAND_ACK acknowledgement{};
        acknowledgement.commandId = 1;
        acknowledgement.command = PAIR;
        acknowledgement.accepted = accepted ? 1 : 0;
        acknowledgement.state = accepted ? 1 : 0;
        send(Packet(STATUS, ACK, acknowledgement), senderIp, senderPort);
        if (!accepted) {
          ESP_LOGW(WIFI_TAG, "Rejected connection request from %s:%u",
                   senderIp, static_cast<unsigned>(senderPort));
        }
        continue;
      }

      if (header.type == COMMAND && header.id == DISCOVERY) {
        if (header.authToken != 0 ||
            header.length != sizeof(DISCOVERY_REQUEST)) {
          continue;
        }
        DISCOVERY_REQUEST request{};
        memcpy(&request, recvBuffer + sizeof(PacketHeader), sizeof(request));
        DISCOVERY_RESPONSE response{};
        response.nonce = request.nonce;
        if (!getLocalAddress(response.ip, sizeof(response.ip), response.port)) {
          continue;
        }
        send(Packet(STATUS, DISCOVERY, response), senderIp, senderPort);
        continue;
      }

      if (!isAuthorizedPeer(senderIp, senderPort, header.authToken)) {
        continue;
      }
      const std::uint32_t sequenceDelta =
          header.seq - lastAuthenticatedSequence;
      if (hasAuthenticatedSequence &&
          (sequenceDelta == 0 || sequenceDelta > 0x7FFFFFFFU)) {
        continue;
      }
      lastAuthenticatedSequence = header.seq;
      hasAuthenticatedSequence = true;

      if (header.type == COMMAND && header.id == ARM) {
        ARM_CMD command{};
        bool accepted = false;
        if (header.length == sizeof(ARM_CMD)) {
          memcpy(&command, recvBuffer + sizeof(PacketHeader), sizeof(command));
          if (command.action == ARM_REQUEST) {
            // Clear an earlier command before opening a new arm lease.
            handleMoveCommand(MOVE_CMD{});
            handleMotionStopCommand();
          }
          accepted = handleArmCommand(command);
          if (accepted && command.action == ARM_DISARM) {
            handleMoveCommand(MOVE_CMD{});
            handleMotionStopCommand();
          }
        }
        COMMAND_ACK acknowledgement{};
        acknowledgement.commandId = command.commandId;
        acknowledgement.command = ARM;
        acknowledgement.accepted = accepted ? 1 : 0;
        acknowledgement.state = driveArmLeaseActive() ? 1 : 0;
        send(Packet(STATUS, ACK, acknowledgement), senderIp, senderPort);
        if (!accepted) {
          ESP_LOGW(WIFI_TAG, "Arm command rejected (action=%u)",
                   static_cast<unsigned>(command.action));
        }
        continue;
      }

      if (header.type == MESSAGE && header.id == STR) {
        if (header.length != sizeof(STR_MSG)) {
          ESP_LOGW(WIFI_TAG, "Dropping debug message with invalid length");
          continue;
        }
        STR_MSG message{};
        memcpy(&message, recvBuffer + sizeof(PacketHeader), sizeof(message));
        message.str[sizeof(message.str) - 1] = '\0';
        ESP_LOGI(WIFI_TAG, "Received debug ping: %s", message.str);
        // Generate a new rover packet so the desktop validates this response
        // against the rover's sequence stream, not the laptop request sequence.
        send(Packet(MESSAGE, STR, message), senderIp, senderPort);
        continue;
      }

      if (header.type == COMMAND && header.id == MOVE) {
        if (header.length != sizeof(MOVE_CMD)) {
          ESP_LOGW(WIFI_TAG, "Dropping movement command with invalid length");
          continue;
        }

        MOVE_CMD command{};
        memcpy(&command, recvBuffer + sizeof(PacketHeader), sizeof(command));
        if (!handleMoveCommand(command)) {
          ESP_LOGW(WIFI_TAG, "Dropping invalid or unqueued movement command");
        } else if (!hasLoggedMove ||
                   command.throttle != lastLoggedMove.throttle ||
                   command.steering != lastLoggedMove.steering ||
                   command.speed != lastLoggedMove.speed) {
          ESP_LOGI(WIFI_TAG, "MOVE throttle=%d steering=%d PWM=%u",
                   static_cast<int>(command.throttle),
                   static_cast<int>(command.steering),
                   static_cast<unsigned>(command.speed));
          lastLoggedMove = command;
          hasLoggedMove = true;
        }
        continue;
      }

      if (header.type == COMMAND && header.id == MOTION) {
        PID_MOTION_CMD command{};
        const bool validLength = header.length == sizeof(PID_MOTION_CMD);
        bool accepted = false;
        if (validLength) {
          memcpy(&command, recvBuffer + sizeof(PacketHeader), sizeof(command));
          accepted = handlePidMotionCommand(command);
        }
        COMMAND_ACK acknowledgement{};
        acknowledgement.commandId = command.commandId;
        acknowledgement.command = MOTION;
        acknowledgement.accepted = accepted ? 1 : 0;
        acknowledgement.state = accepted ? 1 : 0;
        send(Packet(STATUS, ACK, acknowledgement), senderIp, senderPort);
        if (!validLength) {
          ESP_LOGW(WIFI_TAG, "Dropping motion command with invalid length");
          continue;
        }
        if (!accepted) {
          ESP_LOGW(WIFI_TAG, "Dropping invalid motion command");
        } else {
          ESP_LOGI(WIFI_TAG,
                   "PID motion %u kind=%u target=%.2f gains=%.2f/%.2f/%.2f PWM=%u",
                   static_cast<unsigned>(command.commandId),
                   static_cast<unsigned>(command.kind), command.target,
                   command.kp, command.ki, command.kd,
                   static_cast<unsigned>(command.maxSpeed));
        }
        continue;
      }

      if (header.type == COMMAND && header.id == MOTION_STOP) {
        if (header.length != 0 || !handleMotionStopCommand()) {
          ESP_LOGW(WIFI_TAG, "Could not queue motion stop");
        }
        continue;
      }

      if (header.type == COMMAND && header.id == MOTION_KEEPALIVE) {
        if (header.length != sizeof(MOTION_KEEPALIVE_CMD)) {
          ESP_LOGW(WIFI_TAG, "Dropping motion keepalive with invalid length");
          continue;
        }
        MOTION_KEEPALIVE_CMD command{};
        memcpy(&command, recvBuffer + sizeof(PacketHeader), sizeof(command));
        if (!handleMotionKeepalive(command)) {
          ESP_LOGW(WIFI_TAG, "Could not queue motion keepalive");
        }
        continue;
      }

      if (header.type == COMMAND && header.id == RECORDING) {
        RECORDING_CMD command{};
        bool accepted = false;
        if (header.length == sizeof(RECORDING_CMD)) {
          memcpy(&command, recvBuffer + sizeof(PacketHeader), sizeof(command));
          accepted = handleRecordingCommand(command);
        }
        COMMAND_ACK acknowledgement{};
        acknowledgement.commandId = command.commandId;
        acknowledgement.command = RECORDING;
        acknowledgement.accepted = accepted ? 1 : 0;
        acknowledgement.state = getRecordingState();
        if (senderIp[0] != '\0' &&
            command.action != RECORDING_KEEPALIVE) {
          send(Packet(STATUS, ACK, acknowledgement), senderIp,
               ntohs(clientAddr.sin_port));
        }
        if (!accepted) {
          ESP_LOGW(WIFI_TAG, "Recording command rejected (action=%u)",
                   static_cast<unsigned>(command.action));
        }
        continue;
      }

      ESP_LOGW(WIFI_TAG,
               "Ignoring unsupported authorized packet type=%u id=%u",
               static_cast<unsigned>(header.type),
               static_cast<unsigned>(header.id));
    } else {
      ESP_LOGW(WIFI_TAG, "Dropping undersized UDP packet (%d bytes)", bytesReceived);
    }
  }
}

void UDPServer::loopSend() {
  UDPPacket *packet;
  std::uint32_t consecutiveSendErrors = 0;
  while (true) {
    {
      xQueueReceive(toSend, &packet, portMAX_DELAY);

      sockaddr_in sendAddr{};
      sendAddr.sin_family = AF_INET;
      sendAddr.sin_port = htons(packet->peer.port);

      if (inet_pton(AF_INET, packet->peer.ip, &sendAddr.sin_addr) != 1) {
        ESP_LOGE(WIFI_TAG, "Invalid UDP destination IP: %s", packet->peer.ip);
        portENTER_CRITICAL(&droppedPacketMux);
        ++droppedPacketCount;
        portEXIT_CRITICAL(&droppedPacketMux);
        delete packet;
        continue;
      }

      int bytesSent =
          sendto(sock, (const char *)packet->packet.buffer, packet->packet.size,
                 0, reinterpret_cast<sockaddr *>(&sendAddr), sizeof(sendAddr));

      if (bytesSent < 0) {
        const int sendError = errno;
        portENTER_CRITICAL(&droppedPacketMux);
        ++droppedPacketCount;
        portEXIT_CRITICAL(&droppedPacketMux);
        ++consecutiveSendErrors;
        if (consecutiveSendErrors == 1 ||
            consecutiveSendErrors % 100 == 0) {
          ESP_LOGE(WIFI_TAG, "UDP send failed (%u consecutive): errno %d",
                   static_cast<unsigned>(consecutiveSendErrors), sendError);
        }
      } else {
        consecutiveSendErrors = 0;
        PacketHeader header{};
        if (packet->packet.size >= sizeof(header)) {
          memcpy(&header, packet->packet.buffer, sizeof(header));
        }
        // The LiDAR/IMU/thermal stream sends hundreds of datagrams per second.
        // Per-packet serial logs can starve the sender and create queue drops.
        if (header.type != DATA) {
          ESP_LOGI(WIFI_TAG, "Sent %d bytes to %s:%d", bytesSent,
                   packet->peer.ip, packet->peer.port);
        }
      }
      delete packet;
    }
  }
}

UDPServer::UDPServer() {
  if (!initWIFI()) {
    sock = -1;
    return;
  }
  sock = createSocket();
  toSend = xQueueCreate(64, sizeof(UDPPacket *));
  if (toSend == NULL) {
    ESP_LOGE(WIFI_TAG, "Could not allocate UDP transmit queue; network disabled");
    if (sock >= 0) {
      close(sock);
    }
    sock = -1;
  }
}

bool UDPServer::healthy() { return sock >= 0; }

void UDPServer::send(const Packet &packet, const char *ip, unsigned short port) {
  Packet response = packet;
  PacketHeader header{};
  if (response.size >= sizeof(PacketHeader)) {
    memcpy(&header, response.buffer, sizeof(header));
    header.authToken = sessionTokenForPeer(ip, port);
    header.version = PROTOCOL_VERSION;
    memcpy(response.buffer, &header, sizeof(header));
  }
  UDPPacket *p = new (std::nothrow)
      UDPPacket{response, PeerAddress{.ip = "", .port = port}};
  if (p == nullptr) {
    portENTER_CRITICAL(&droppedPacketMux);
    ++droppedPacketCount;
    portEXIT_CRITICAL(&droppedPacketMux);
    return;
  }
  snprintf(p->peer.ip, sizeof(p->peer.ip), "%s", ip);
  if (toSend == nullptr || xQueueSend(toSend, &p, 0) != pdTRUE) {
    delete p;
    portENTER_CRITICAL(&droppedPacketMux);
    ++droppedPacketCount;
    portEXIT_CRITICAL(&droppedPacketMux);
  }
}

std::uint32_t UDPServer::getDroppedPacketCount() {
  portENTER_CRITICAL(&droppedPacketMux);
  const std::uint32_t count = droppedPacketCount;
  portEXIT_CRITICAL(&droppedPacketMux);
  return count;
}

bool UDPServer::getStatusDestination(char *ip, std::size_t ipCapacity,
                                     unsigned short &port) {
  if (ip == nullptr || ipCapacity == 0) return false;

  bool available = false;
  portENTER_CRITICAL(&statusDestinationMux);
  if (statusDestinationPort != 0 && statusDestinationIp[0] != '\0') {
    snprintf(ip, ipCapacity, "%s", statusDestinationIp);
    port = statusDestinationPort;
    available = true;
  }
  portEXIT_CRITICAL(&statusDestinationMux);
  return available;
}

bool UDPServer::connectPeer(std::uint32_t token, const char *ip,
                         unsigned short port) {
  if (token == 0 || ip == nullptr || ip[0] == '\0' ||
      port == 0) {
    return false;
  }

  portENTER_CRITICAL(&statusDestinationMux);
  sessionToken = token;
  snprintf(statusDestinationIp, sizeof(statusDestinationIp), "%s", ip);
  statusDestinationPort = port;
  portEXIT_CRITICAL(&statusDestinationMux);
  ESP_LOGI(WIFI_TAG, "Laptop connected at %s:%u", ip,
           static_cast<unsigned>(port));
  return true;
}

bool UDPServer::isPairedEndpoint(const char *ip, unsigned short port) {
  return sessionTokenForPeer(ip, port) != 0;
}

std::uint32_t UDPServer::sessionTokenForPeer(const char *ip, unsigned short port) {
  if (ip == nullptr || ip[0] == '\0' || port == 0) return 0;
  std::uint32_t token = 0;
  portENTER_CRITICAL(&statusDestinationMux);
  if (statusDestinationPort == port &&
      strncmp(statusDestinationIp, ip, sizeof(statusDestinationIp)) == 0) {
    token = sessionToken;
  }
  portEXIT_CRITICAL(&statusDestinationMux);
  return token;
}

bool UDPServer::isAuthorizedPeer(const char *ip, unsigned short port,
                                 std::uint32_t authToken) {
  return authToken != 0 && authToken == sessionTokenForPeer(ip, port);
}
