#pragma once

#include "plugins/ftp_server/ftp_server.h"

#include <string>
#include <mutex>
#include "server_plugin_impl_base.h"

namespace mavsdk {

class FtpServerImpl : public ServerPluginImplBase {
public:
    explicit FtpServerImpl(std::shared_ptr<ServerComponent> server_component);

    ~FtpServerImpl() override;

    void init() override;
    void deinit() override;

    FtpServer::Result provide_file(const std::string& path);

private:
    std::mutex _saved_paths_mutex{};
    std::vector<std::string> _saved_paths{};
};

} // namespace mavsdk
