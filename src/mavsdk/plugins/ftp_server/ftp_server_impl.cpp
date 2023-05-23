#include "ftp_server_impl.h"

namespace mavsdk {

FtpServerImpl::FtpServerImpl(std::shared_ptr<ServerComponent> server_component) :
    ServerPluginImplBase(server_component)
{
    _server_component_impl->register_plugin(this);
}

FtpServerImpl::~FtpServerImpl()
{
    _server_component_impl->unregister_plugin(this);
}

void FtpServerImpl::init() {}

void FtpServerImpl::deinit() {}

FtpServer::Result FtpServerImpl::provide_file(const std::string& path)
{
    std::lock_guard<std::mutex> lock(_saved_paths_mutex);

    const auto found =
        std::find_if(_saved_paths.begin(), _saved_paths.end(), [&path](const std::string& item) {
            return item == path;
        }) != _saved_paths.end();

    if (found) {
        return FtpServer::Result::Duplicate;
    }

    _saved_paths.push_back(path);

    return FtpServer::Result::Success;
}

} // namespace mavsdk
