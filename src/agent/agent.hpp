#pragma once

#include "../ops/ops.hpp"

#include <string>
#include <utility>
#include <vector>

namespace kiln {

struct AgentReply {
    bool ok = true;
    std::string text;
    nlohmann::json ops = nlohmann::json::array();
};

struct AgentConfig {
    std::string api_key;
    std::string api_base = "https://api.openai.com/v1";
    std::string model = "gpt-4.1-mini";
};

AgentConfig agent_config_from_env();
AgentReply interpret(const std::string& input, Document& doc, const AgentConfig& cfg);
std::string help_text();
bool named_color(const std::string& name, Vec3& out);

}  // namespace kiln
