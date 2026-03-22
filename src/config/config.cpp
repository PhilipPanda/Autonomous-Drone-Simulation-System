#include "adsim/config/config.hpp"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <stdexcept>
#include <cctype>

namespace adsim {

namespace {

std::string trim(const std::string& s) {
    auto start = s.begin();
    while (start != s.end() && std::isspace(static_cast<unsigned char>(*start)))
        ++start;
    auto end = s.end();
    while (end != start && std::isspace(static_cast<unsigned char>(*(end - 1))))
        --end;
    return {start, end};
}

}

Config Config::parse(const std::string& content) {
    Config cfg;
    std::string current_section;
    std::istringstream stream(content);
    std::string line;

    while (std::getline(stream, line)) {
        auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }
        line = trim(line);

        if (line.empty()) continue;

        if (line.front() == '[' && line.back() == ']') {
            current_section = trim(line.substr(1, line.size() - 2));
            continue;
        }

        auto eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = trim(line.substr(0, eq));
        std::string value = trim(line.substr(eq + 1));

        if (!key.empty()) {
            cfg.data_[current_section][key] = value;
        }
    }

    return cfg;
}

Config Config::from_file(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw ConfigError("Cannot open config file: " + path);
    }
    std::string content((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
    return parse(content);
}

Config Config::from_string(const std::string& content) {
    return parse(content);
}

bool Config::has(const std::string& section, const std::string& key) const {
    auto sit = data_.find(section);
    if (sit == data_.end()) return false;
    return sit->second.count(key) > 0;
}

std::vector<std::string> Config::sections() const {
    std::vector<std::string> result;
    for (const auto& [section, _] : data_) {
        result.push_back(section);
    }
    return result;
}

double Config::get_double(const std::string& section, const std::string& key, double default_val) const {
    if (!has(section, key)) return default_val;
    try {
        return std::stod(data_.at(section).at(key));
    } catch (...) {
        return default_val;
    }
}

int Config::get_int(const std::string& section, const std::string& key, int default_val) const {
    if (!has(section, key)) return default_val;
    try {
        return std::stoi(data_.at(section).at(key));
    } catch (...) {
        return default_val;
    }
}

std::string Config::get_string(const std::string& section, const std::string& key, const std::string& default_val) const {
    if (!has(section, key)) return default_val;
    return data_.at(section).at(key);
}

bool Config::get_bool(const std::string& section, const std::string& key, bool default_val) const {
    if (!has(section, key)) return default_val;
    const std::string& val = data_.at(section).at(key);
    return val == "true" || val == "1" || val == "yes";
}

}
