#pragma once
#include <string>
#include <functional>
#include <thread>
#include <atomic>

// ============================================================
//  SOCKET SERVER
//  Streams simulation state as JSON over TCP port 9000
//  Python dashboard connects and reads one packet per tick
// ============================================================
class SocketServer {
public:
    SocketServer(int port = 9000);
    ~SocketServer();

    // Start listening in background thread
    void start();

    // Send a JSON string to all connected clients
    void broadcast(const std::string& json);

    // Stop the server
    void stop();

    bool isRunning() const { return running_; }

private:
    int port_;
    int serverFd_;
    int clientFd_;
    std::atomic<bool> running_;
    std::thread acceptThread_;

    void acceptLoop();
};