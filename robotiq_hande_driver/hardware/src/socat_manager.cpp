#include "robotiq_hande_driver/socat_manager.hpp"

#include <stdexcept>

namespace robotiq_hande_driver {

SocatManager::SocatManager(const std::string& host, int port, const std::string& tty_path)
    : host_{host}, port_{port}, tty_path_{tty_path} {}

SocatManager::~SocatManager() {
    stop();
}

void SocatManager::start() {
    // Fork to create child process
    socat_pid_ = fork();

    // Child process code
    if(socat_pid_ == 0) {
        // Child process - execute socat
        std::string pty_endpoint = "pty,link=" + tty_path_ + ",raw,ignoreeof,waitslave";
        std::string tcp_endpoint = "tcp:" + host_ + ":" + std::to_string(port_);

        char* args[] = {
            const_cast<char*>("socat"),
            const_cast<char*>(pty_endpoint.c_str()),
            const_cast<char*>(tcp_endpoint.c_str()),
            nullptr};

        execvp("socat", args);
        perror("execvp socat failed");
        exit(1);
    }

    // Parent process code
    started_ = true;

    // Check if process is still alive
    int status;
    if(waitpid(socat_pid_, &status, WNOHANG) == 0) return;

    throw std::runtime_error("Failed to start the forked process for virtual serial port (socat).");
}

void SocatManager::stop() {
    if(!started_ || socat_pid_ == -1) return;

    kill(socat_pid_, SIGTERM);

    int status;
    waitpid(socat_pid_, &status, 0);

    started_ = false;
    unlink(tty_path_.c_str());
    return;
}

bool SocatManager::is_alive() const {
    if(!started_) return false;

    int status;
    return waitpid(socat_pid_, &status, WNOHANG) == 0;
}

}  // namespace robotiq_hande_driver
