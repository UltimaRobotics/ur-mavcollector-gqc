#pragma once

#include <string>
#include <map>
#include <variant>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

/// Simple JSON configuration parser for MAVLink Data Collector
/// Supports basic JSON structure with string, integer, and boolean values
class JsonConfig
{
public:
    using Value = std::variant<std::string, int, bool, double>;
    
    JsonConfig() = default;
    ~JsonConfig() = default;
    
    /// Load JSON configuration from file
    /// @param filePath Path to JSON configuration file
    /// @return true if loaded successfully
    bool loadFromFile(const std::string& filePath);
    
    /// Get string value
    /// @param key Configuration key
    /// @param defaultValue Default value if key not found
    /// @return String value
    std::string getString(const std::string& key, const std::string& defaultValue = "") const;
    
    /// Get integer value
    /// @param key Configuration key
    /// @param defaultValue Default value if key not found
    /// @return Integer value
    int getInt(const std::string& key, int defaultValue = 0) const;
    
    /// Get boolean value
    /// @param key Configuration key
    /// @param defaultValue Default value if key not found
    /// @return Boolean value
    bool getBool(const std::string& key, bool defaultValue = false) const;
    
    /// Get double value
    /// @param key Configuration key
    /// @param defaultValue Default value if key not found
    /// @return Double value
    double getDouble(const std::string& key, double defaultValue = 0.0) const;
    
    /// Check if key exists
    /// @param key Configuration key
    /// @return true if key exists
    bool hasKey(const std::string& key) const;
    
    /// Get all keys
    /// @return Vector of all configuration keys
    std::vector<std::string> getKeys() const;
    
    /// Print configuration (for debugging)
    void printConfig() const;

private:
    std::map<std::string, Value> _values;
    
    /// Parse JSON string
    /// @param jsonString JSON string to parse
    /// @return true if parsed successfully
    bool parseJson(const std::string& jsonString);
    
    /// Trim whitespace from string
    /// @param str String to trim
    /// @return Trimmed string
    std::string trim(const std::string& str) const;
    
    /// Parse value from JSON token
    /// @param token JSON token string
    /// @return Parsed value
    Value parseValue(const std::string& token) const;
    
    /// Extract key from JSON line
    /// @param line JSON line
    /// @return Extracted key
    std::string extractKey(const std::string& line) const;
    
    /// Extract value from JSON line
    /// @param line JSON line
    /// @return Extracted value token
    std::string extractValue(const std::string& line) const;
};
