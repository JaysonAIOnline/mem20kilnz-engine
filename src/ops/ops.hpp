#pragma once

#include "../scene/scene.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace kiln {

struct OpResult {
    bool ok = true;
    std::string message;
    nlohmann::json data = nlohmann::json::object();
};

struct Document {
    Scene scene;
    std::vector<nlohmann::json> undo;
    std::vector<nlohmann::json> redo;
    int max_undo = 80;

    void push_undo();
    bool undo_once();
    bool redo_once();
    void reset();
};

OpResult apply_op(Document& doc, const nlohmann::json& op);
OpResult apply_ops(Document& doc, const nlohmann::json& ops);

int ensure_default_camera(Scene& s);
int ensure_default_sun(Scene& s);

}  // namespace kiln
