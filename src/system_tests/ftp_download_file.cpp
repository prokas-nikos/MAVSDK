#include "log.h"
#include "mavsdk.h"
#include <filesystem>
#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <fstream>
#include "plugins/ftp/ftp.h"
#include "plugins/ftp_server/ftp_server.h"

using namespace mavsdk;

static constexpr double reduced_timeout_s = 0.1;

// TODO: make this compatible for Windows using GetTempPath2

namespace fs = std::filesystem;

static auto sep = fs::path::preferred_separator;

static const fs::path temp_dir_provided = "/tmp/mavsdk_systemtest_temp_data/provided";
static const fs::path temp_file_provided = "data.bin";

static const fs::path temp_dir_downloaded = "/tmp/mavsdk_systemtest_temp_data/downloaded";

static bool create_temp_file(const fs::path& path, size_t len)
{
    std::vector<uint8_t> data;
    data.reserve(len);

    for (size_t i = 0; i < len; ++i) {
        data.push_back(static_cast<uint8_t>(i % 256));
    }

    const auto parent_path = path.parent_path();
    create_directories(parent_path);

    std::ofstream tempfile;
    tempfile.open(path, std::ios::out | std::ios::binary);
    tempfile.write((const char*)data.data(), data.size());
    tempfile.close();

    return true;
}

static bool reset_directories(const fs::path& path)
{
    std::error_code ec;
    fs::remove_all(path, ec);

    return fs::create_directories(path);
}

TEST(SystemTest, FtpDownloadFile)
{
    Mavsdk mavsdk_groundstation;
    mavsdk_groundstation.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::GroundStation});
    mavsdk_groundstation.set_timeout_s(reduced_timeout_s);

    Mavsdk mavsdk_autopilot;
    mavsdk_autopilot.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::Autopilot});
    mavsdk_autopilot.set_timeout_s(reduced_timeout_s);

    ASSERT_EQ(mavsdk_groundstation.add_any_connection("udp://:17000"), ConnectionResult::Success);
    ASSERT_EQ(
        mavsdk_autopilot.add_any_connection("udp://127.0.0.1:17000"), ConnectionResult::Success);

    auto ftp_server = FtpServer{
        mavsdk_autopilot.server_component_by_type(Mavsdk::ServerComponentType::Autopilot)};

    auto maybe_system = mavsdk_groundstation.first_autopilot(10.0);
    ASSERT_TRUE(maybe_system);
    auto system = maybe_system.value();

    ASSERT_TRUE(system->has_autopilot());

    ASSERT_TRUE(create_temp_file(temp_dir_provided / temp_file_provided, 50));
    ASSERT_TRUE(reset_directories(temp_dir_downloaded));

    auto ftp = Ftp{system};

    auto prom = std::promise<Ftp::Result>();
    auto fut = prom.get_future();
    ftp.download_async(
        temp_dir_provided / temp_file_provided,
        temp_dir_downloaded,
        [&prom](Ftp::Result result, Ftp::ProgressData progress_data) {
            if (result != Ftp::Result::Next) {
                prom.set_value(result);
            } else {
                LogDebug() << "Download progress: " << progress_data.bytes_transferred << "/"
                           << progress_data.total_bytes << " bytes";
            }
        });

    auto future_status = fut.wait_for(std::chrono::seconds(1));
    ASSERT_EQ(future_status, std::future_status::ready);
    EXPECT_EQ(fut.get(), Ftp::Result::Success);
}

TEST(SystemTest, FtpDownloadFileLossy)
{
#if 0
    Mavsdk mavsdk_groundstation;
    mavsdk_groundstation.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::GroundStation});
    mavsdk_groundstation.set_timeout_s(reduced_timeout_s);

    Mavsdk mavsdk_autopilot;
    mavsdk_autopilot.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::Autopilot});
    mavsdk_autopilot.set_timeout_s(reduced_timeout_s);

    // Drop every third message
    unsigned counter = 0;
    auto drop_some = [&counter](mavlink_message_t&) { return counter++ % 5; };

    mavsdk_groundstation.intercept_incoming_messages_async(drop_some);
    mavsdk_groundstation.intercept_incoming_messages_async(drop_some);

    ASSERT_EQ(mavsdk_groundstation.add_any_connection("udp://:17000"), ConnectionResult::Success);
    ASSERT_EQ(
        mavsdk_autopilot.add_any_connection("udp://127.0.0.1:17000"), ConnectionResult::Success);

    auto param_server = ParamServer{
        mavsdk_autopilot.server_component_by_type(Mavsdk::ServerComponentType::Autopilot)};

    auto maybe_system = mavsdk_groundstation.first_autopilot(10.0);
    ASSERT_TRUE(maybe_system);
    auto system = maybe_system.value();

    ASSERT_TRUE(system->has_autopilot());

    const auto test_float_params = generate_float_params();
    const auto test_int_params = generate_int_params();
    const auto test_string_params = generate_string_params();

    // Add many params (these don't need extended)
    for (auto const& [key, val] : test_float_params) {
        EXPECT_EQ(param_server.provide_param_float(key, val), ParamServer::Result::Success);
    }
    for (auto const& [key, val] : test_int_params) {
        EXPECT_EQ(param_server.provide_param_int(key, val), ParamServer::Result::Success);
    }

    for (auto const& [key, val] : test_string_params) {
        EXPECT_EQ(param_server.provide_param_custom(key, val), ParamServer::Result::Success);
    }

    {
        auto param_sender = Param{system};
        // Here we use the non-extended protocol
        param_sender.select_component(1, Param::ProtocolVersion::V1);
        const auto all_params = param_sender.get_all_params();
        assert_equal<int, Param::IntParam>(test_int_params, all_params.int_params);
        assert_equal<float, Param::FloatParam>(test_float_params, all_params.float_params);
    }
    {
        auto param_sender = Param{system};
        // now we do the same, but this time with the extended protocol
        param_sender.select_component(1, Param::ProtocolVersion::Ext);
        const auto all_params = param_sender.get_all_params();
        assert_equal<int, Param::IntParam>(test_int_params, all_params.int_params);
        assert_equal<float, Param::FloatParam>(test_float_params, all_params.float_params);
        assert_equal<std::string, Param::CustomParam>(test_string_params, all_params.custom_params);
    }

    // Before going out of scope, we need to make sure to no longer access the
    // drop_some callback which accesses the local counter variable.
    mavsdk_groundstation.intercept_incoming_messages_async(nullptr);
    mavsdk_groundstation.intercept_incoming_messages_async(nullptr);
#endif
}
