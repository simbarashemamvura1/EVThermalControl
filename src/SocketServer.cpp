#include "SocketServer.h"
#include <iostream>
#include <cstring>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
  typedef int socklen_t;
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <unistd.h>
  #define INVALID_SOCKET -1
  #define SOCKET_ERROR   -1
  #define closesocket    close
#endif

SocketServer::SocketServer(int port)
    : port_(port), serverFd_(INVALID_SOCKET),
      clientFd_(INVALID_SOCKET), running_(false) {
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2,2), &wsa);
#endif
}

SocketServer::~SocketServer() {
    stop();
#ifdef _WIN32
    WSACleanup();
#endif
}

void SocketServer::start() {
    serverFd_ = (int)socket(AF_INET, SOCK_STREAM, 0);
    if (serverFd_ == INVALID_SOCKET) {
        std::cerr << "[SOCKET] Failed to create socket\n";
        return;
    }

    int opt = 1;
    setsockopt(serverFd_, SOL_SOCKET, SO_REUSEADDR,
               (const char*)&opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(port_);

    if (bind(serverFd_, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        std::cerr << "[SOCKET] Bind failed on port " << port_ << "\n";
        return;
    }

    listen(serverFd_, 1);
    running_ = true;
    std::cout << "[SOCKET] Server listening on port " << port_ << "\n";

    acceptThread_ = std::thread(&SocketServer::acceptLoop, this);
    acceptThread_.detach();
}

void SocketServer::acceptLoop() {
    while (running_) {
        sockaddr_in clientAddr{};
        socklen_t clientLen = sizeof(clientAddr);
        int fd = (int)accept(serverFd_,
                             (sockaddr*)&clientAddr, &clientLen);
        if (fd == INVALID_SOCKET) continue;
        std::cout << "[SOCKET] Client connected\n";
        if (clientFd_ != INVALID_SOCKET)
            closesocket(clientFd_);
        clientFd_ = fd;
    }
}

void SocketServer::broadcast(const std::string& json) {
    if (clientFd_ == INVALID_SOCKET) return;
    std::string msg = json + "\n";
    int sent = send(clientFd_, msg.c_str(), (int)msg.size(), 0);
    if (sent == SOCKET_ERROR) {
        closesocket(clientFd_);
        clientFd_ = INVALID_SOCKET;
        std::cout << "[SOCKET] Client disconnected\n";
    }
}

void SocketServer::stop() {
    running_ = false;
    if (clientFd_ != INVALID_SOCKET) {
        closesocket(clientFd_);
        clientFd_ = INVALID_SOCKET;
    }
    if (serverFd_ != INVALID_SOCKET) {
        closesocket(serverFd_);
        serverFd_ = INVALID_SOCKET;
    }
}