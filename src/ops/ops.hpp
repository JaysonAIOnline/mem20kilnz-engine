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

    /// Every op that was applied, in order, with its verdict. This is the
    /// provenance record: a client cannot otherwise see which ops a language
    /// model produced on its own, because those never pass through the RPC
    /// layer as separate calls.
    std::vector<nlohmann::json> journal;
    int max_journal = 4096;

    void push_undo();
    bool undo_once();
    bool redo_once();
    void reset();
    void record(const nlohmann::json& op, const OpResult& r);
};

OpResult apply_op_impl(Document& doc, const nlohmann::json& op);
OpResult apply_op(Document& doc, const nlohmann::json& op);
OpResult apply_ops(Document& doc, const nlohmann::json& ops);

int ensure_default_camera(Scene& s);
int ensure_default_sun(Scene& s);

}  // namespace kiln
