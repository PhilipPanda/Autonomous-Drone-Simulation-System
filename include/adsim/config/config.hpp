#pragma once

#include <map>
#include <string>
#include <stdexcept>
#include <vector>

namespace adsim {

class Config {
public:
    static Config from_file(const std::string& path);
    static Config from_string(const std::string& content);

    double get_double(const std::string& section, const std::string& key, double default_val = 0.0) const;
    int get_int(const std::string& section, const std::string& key, int default_val = 0) const;
    std::string get_string(const std::string& section, const std::string& key, const std::string& default_val = "") const;
    bool get_bool(const std::string& section, const std::string& key, bool default_val = false) const;

    bool has(const std::string& section, const std::string& key) const;
    std::vector<std::string> sections() const;

private:
    std::map<std::string, std::map<std::string, std::string>> data_;

    static Config parse(const std::string& content);
};

struct ConfigError : std::runtime_error {
    explicit ConfigError(const std::string& msg) : std::runtime_error(msg) {}
};

}
