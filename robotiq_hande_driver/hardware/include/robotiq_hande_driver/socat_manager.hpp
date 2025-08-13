#ifndef ROBOTIQ_HANDE_DRIVER__SOCAT_MANAGER_HPP_
#define ROBOTIQ_HANDE_DRIVER__SOCAT_MANAGER_HPP_

#include <chrono>
#include <cstdlib>
#include <exception>
#include <string>
#include <thread>

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

namespace robotiq_hande_driver {

static constexpr auto WAIT_FOR_SOCAT = std::chrono::milliseconds(100);

class SocatManager {
   public:
    SocatManager(const std::string& host, int port, const std::string& tty_path);

    ~SocatManager();

    void start();

    void stop();

    bool is_alive() const;

   private:
    pid_t socat_pid_;
    bool started_;

    std::string host_;
    int port_;
    std::string tty_path_;
};

}  // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__SOCAT_MANAGER_HPP_
