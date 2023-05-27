#include "fs_helpers.h"
#include <filesystem>
#include <fstream>
#include <vector>
#include <cstring>


bool create_temp_file(const fs::path& path, size_t len)
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

bool reset_directories(const fs::path& path)
{
    std::error_code ec;
    fs::remove_all(path, ec);

    return fs::create_directories(path);
}

bool are_files_identical(const fs::path& path1, const fs::path& path2)
{
    std::ifstream file1(path1, std::ios::binary);
    std::ifstream file2(path2, std::ios::binary);

    if (!file1 || !file2) {
        // Failed to open one or both files
        return false;
    }

    // Read the files' contents and compare them
    char buffer1[4096];
    char buffer2[4096];

    do {
        file1.read(buffer1, sizeof(buffer1));
        file2.read(buffer2, sizeof(buffer2));

        if (file1.gcount() != file2.gcount() || std::memcmp(buffer1, buffer2, file1.gcount()) != 0) {
            // Files are not identical
            return false;
        }
    } while (file1.good() || file2.good());

    // Make sure both files reached the end at the same time
    return !file1.good() && !file2.good();
}

bool file_exists(const fs::path& path)
{
    return fs::exists(path);
}
