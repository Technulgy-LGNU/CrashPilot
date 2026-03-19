#include <iostream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>

int main() {
    const char* socket_path = "/tmp/rust_to_cpp.sock";

    // Remove existing socket file
    unlink(socket_path);

    int server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "Failed to create socket\n";
        return 1;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path) - 1);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Bind failed\n";
        return 1;
    }

    if (listen(server_fd, 5) < 0) {
        std::cerr << "Listen failed\n";
        return 1;
    }

    std::cout << "C++ server listening on " << socket_path << std::endl;

    int client_fd = accept(server_fd, nullptr, nullptr);
    if (client_fd < 0) {
        std::cerr << "Accept failed\n";
        return 1;
    }

    char buffer[1024];
    int n;
    while ((n = read(client_fd, buffer, sizeof(buffer))) > 0) {
        std::cout << "Received: " << std::string(buffer, n) << std::endl;
    }

    close(client_fd);
    close(server_fd);
    unlink(socket_path);

    return 0;
}