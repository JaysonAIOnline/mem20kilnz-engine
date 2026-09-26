#include "agent.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <unordered_map>
#include <vector>

namespace kiln {
namespace {

std::string lower_copy(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string trim(std::string s) {
    auto notsp = [](int ch) { return !std::isspace(ch); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notsp));
    s.erase(std::find_if(s.rbegin(), s.rend(), notsp).base(), s.end());
    return s;
}

std::vector<std::string> split_ws(const std::string& s) {
    std::istringstream in(s);
    std::vector<std::string> t;
    std::string x;
    while (in >> x) t.push_back(x);
    return t;
}

bool is_number(const std::string& s) {
    if (s.empty()) return false;
    char* end = nullptr;
    std::strtod(s.c_str(), &end);
    return end && *end == 0;
}

float to_f(const std::string& s, float d = 0) {
    if (!is_number(s)) return d;
    return std::strtof(s.c_str(), nullptr);
}

bool parse_vec3_tokens(const std::vector<std::string>& t, size_t i, Vec3& v) {
    if (i + 2 >= t.size()) return false;
    if (!is_number(t[i]) || !is_number(t[i + 1]) || !is_number(t[i + 2])) return false;
    v = {to_f(t[i]), to_f(t[i + 1]), to_f(t[i + 2])};
    return true;
}

nlohmann::json vecj(Vec3 v) { return nlohmann::json::array({v.x, v.y, v.z}); }

const char* kSystemPrompt = R"(You are kiln, a Linux-native 3D modeler. Convert the user's request into a JSON array of operations. Output JSON only.

Each op is an object with field "op". Supported ops:
- {"op":"create","primitive":"cube|plane|sphere|cylinder|cone|torus|camera|light|sun|point|empty","name":"Box","size":[1,1,1],"translation":[0,0,0],"rotation":[0,0,0],"scale":[1,1,1],"albedo":[r,g,b],"roughness":0.45,"metallic":0}
- {"op":"delete","target":"Box"}
- {"op":"duplicate","target":"Box"}
- {"op":"rename","target":"Box","to":"Hero"}
- {"op":"select","target":"Box"}
- {"op":"move","target":"Box","translation":[x,y,z]}
- {"op":"translate","target":"Box","delta":[dx,dy,dz]}
- {"op":"rotate","target":"Box","rotation":[x,y,z]}  (degrees, absolute) or {"delta":[...]}
- {"op":"scale","target":"Box","scale":[x,y,z]}
- {"op":"color","target":"Box","albedo":[r,g,b],"roughness":0.4,"metallic":0.1}
- {"op":"parent","target":"Child","parent":"Parent"}
- {"op":"lookat","eye":[x,y,z],"center":[x,y,z]}
- {"op":"set_camera","target":"Camera","fov":50}
- {"op":"select_faces","target":"Box","region":"top|bottom|left|right|front|back|all"} or {"faces":[0,1]}
- {"op":"extrude","amount":0.5,"region":"top"}
- {"op":"inset","amount":0.15,"region":"top"}
- {"op":"delete_faces","region":"bottom"}
- {"op":"mode","mode":"face|edge|vert|object"}
- {"op":"select_edges","region":"top"} or {"axis":"y"} or {"edges":[[0,1]]}
- {"op":"select_edge_loop","region":"top"}
- {"op":"select_edge_ring","axis":"y"}
- {"op":"select_verts","region":"top"} or {"verts":[0,1,2]}
- {"op":"loop_cut","cuts":1,"axis":"y"}
- {"op":"bevel","amount":0.12}
- {"op":"subdivide","cuts":1,"region":"all"}
- {"op":"merge"}
- {"op":"fill"}
- {"op":"mirror","axis":"x"}
- {"op":"solidify","thickness":0.08}
- {"op":"array","count":4,"axis":"x","spacing":1.2} or {"offset":[1.2,0,0]}
- {"op":"spin","steps":8,"angle":360,"axis":"y"}
- {"op":"bridge"}
- {"op":"join","other":"Sphere"}
- {"op":"ground"}
- {"op":"origin"}
- {"op":"triangulate"}
- {"op":"poke"}
- {"op":"flip"}
- {"op":"dissolve"}
- {"op":"duplicate_faces","amount":0.2}
- {"op":"separate"}
- {"op":"weld","distance":0.001}
- {"op":"smooth","factor":0.5,"iters":2}
- {"op":"push","amount":0.1}
- {"op":"sphere","factor":1}
- {"op":"flatten","axis":"y"}
- {"op":"randomize","amount":0.05}
- {"op":"apply"}
- {"op":"bisect","axis":"y","distance":0}
- {"op":"snap","grid":0.1}
- {"op":"split"}
- {"op":"connect"}
- {"op":"grow"}
- {"op":"invert"}
- {"op":"linked"}
- {"op":"rig","kind":"biped|quadruped|face","pose":"tpose|apose"}
- {"op":"bind"}
- {"op":"pose","target":"UpperArm.L","rotation":[0,0,40]}
- {"op":"rest"}
- {"op":"shape","name":"smile","value":1}
- {"op":"lod"} or {"op":"lod","level":1}
- {"op":"describe"}
- {"op":"translate","delta":[0,0.2,0]}  (moves selected verts in edit mode)
- {"op":"new"}
- {"op":"undo"}
- {"op":"redo"}

Y is up. A cube of size 1 sitting on the ground has translation [0,0.5,0].
To extrude a side: select_faces with a region, then extrude. Prefer few ops. Name objects clearly.)";

std::string shell_quote(const std::string& s) {
    std::string o = "'";
    for (char c : s) {
        if (c == '\'') o += "'\\''";
        else o += c;
    }
    o += "'";
    return o;
}

std::string run_curl(const std::string& cmd) {
    FILE* p = popen(cmd.c_str(), "r");
    if (!p) return {};
    std::string out;
    char buf[4096];
    while (fgets(buf, sizeof(buf), p)) out += buf;
    pclose(p);
    return out;
}

bool extract_json_array(const std::string& s, nlohmann::json& out) {
    auto a = s.find('[');
    auto b = s.rfind(']');
    if (a == std::string::npos || b == std::string::npos || b <= a) return false;
    try {
        out = nlohmann::json::parse(s.substr(a, b - a + 1));
        return out.is_array();
    } catch (...) {
        return false;
    }
}

AgentReply apply_and_reply(Document& doc, const nlohmann::json& ops) {
    AgentReply r;
    r.ops = ops.is_array() ? ops : nlohmann::json::array({ops});
    OpResult orr = apply_ops(doc, ops);
    r.ok = orr.ok;
    r.text = orr.message;
    return r;
}

bool dsl_command(const std::string& line, nlohmann::json& ops) {
    auto t = split_ws(line);
    if (t.empty()) return false;
    std::string c = lower_copy(t[0]);
    auto rest_name = [&](size_t i) -> std::string {
        if (i >= t.size()) return {};
        return t[i];
    };

    if (c == "help" || c == "list" || c == "info" || c == "preview" || c == "export" || c == "quit" ||
        c == "exit" || c == "save" || c == "open" || c == "load")
        return false;

    if (c == "new" || c == "reset" || c == "clear") {
        ops = nlohmann::json::array({{{"op", "new"}}});
        return true;
    }
    if (c == "undo") {
        ops = nlohmann::json::array({{{"op", "undo"}}});
        return true;
    }
    if (c == "redo") {
        ops = nlohmann::json::array({{{"op", "redo"}}});
        return true;
    }
    if (c == "create" || c == "add" || c == "mk") {
        if (t.size() < 2) return false;
        std::string primtok = lower_copy(t[1]);
        PrimType dummy;
        if (!parse_prim(primtok, dummy) && primtok != "camera" && primtok != "light" &&
            primtok != "sun" && primtok != "point" && primtok != "empty")
            return false;
        nlohmann::json op = {{"op", "create"}, {"primitive", primtok}};
        size_t i = 2;
        if (i < t.size() && !is_number(t[i]) && lower_copy(t[i]) != "size") {
            op["name"] = t[i];
            ++i;
        }
        Vec3 v;
        while (i < t.size()) {
            std::string k = lower_copy(t[i]);
            if ((k == "at" || k == "pos" || k == "position") && parse_vec3_tokens(t, i + 1, v)) {
                op["translation"] = vecj(v);
                i += 4;
            } else if (k == "size" && parse_vec3_tokens(t, i + 1, v)) {
                op["size"] = vecj(v);
                i += 4;
            } else if ((k == "rot" || k == "rotation") && parse_vec3_tokens(t, i + 1, v)) {
                op["rotation"] = vecj(v);
                i += 4;
            } else if (k == "color" || k == "albedo") {
                Vec3 col;
                if (i + 1 < t.size() && named_color(t[i + 1], col)) {
                    op["albedo"] = vecj(col);
                    i += 2;
                } else if (parse_vec3_tokens(t, i + 1, col)) {
                    op["albedo"] = vecj(col);
                    i += 4;
                } else
                    ++i;
            } else if (parse_vec3_tokens(t, i, v) && !op.contains("translation")) {
                op["translation"] = vecj(v);
                i += 3;
            } else
                ++i;
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if ((c == "delete" || c == "rm" || c == "remove") && t.size() >= 2 &&
        (lower_copy(t[1]) == "faces" || lower_copy(t[1]) == "face")) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "delete_faces";
        if (t.size() >= 3) op["region"] = lower_copy(t[2]);
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "delete" || c == "rm" || c == "remove") {
        ops = nlohmann::json::array({{{"op", "delete"}, {"target", rest_name(1)}}});
        return t.size() >= 2;
    }
    if (c == "mode" && t.size() >= 2) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "mode";
        op["mode"] = lower_copy(t[1]);
        ops = nlohmann::json::array({op});
        return true;
    }
    auto is_region = [](const std::string& r) {
        return r == "top" || r == "bottom" || r == "left" || r == "right" || r == "front" ||
               r == "back" || r == "up" || r == "down" || r == "all" || r == "+x" || r == "-x" ||
               r == "+y" || r == "-y" || r == "+z" || r == "-z";
    };
    if (c == "extrude") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "extrude";
        for (size_t i = 1; i < t.size(); ++i) {
            std::string k = lower_copy(t[i]);
            if (is_region(k)) op["region"] = k;
            else if (is_number(t[i])) op["amount"] = to_f(t[i]);
            else if (k != "by" && k != "the" && k != "of" && k != "face" && k != "faces") return false;
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "inset") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "inset";
        for (size_t i = 1; i < t.size(); ++i) {
            std::string k = lower_copy(t[i]);
            if (is_region(k)) op["region"] = k;
            else if (is_number(t[i])) op["amount"] = to_f(t[i]);
            else if (k != "by" && k != "the" && k != "of" && k != "face" && k != "faces") return false;
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if ((c == "duplicate" || c == "dup") && t.size() >= 2 &&
        (lower_copy(t[1]) == "faces" || lower_copy(t[1]) == "face")) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "duplicate_faces";
        for (size_t i = 2; i < t.size(); ++i) {
            if (is_number(t[i])) op["amount"] = to_f(t[i]);
            else if (is_region(lower_copy(t[i]))) op["region"] = lower_copy(t[i]);
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "duplicate" || c == "dup" || c == "copy") {
        ops = nlohmann::json::array({{{"op", "duplicate"}, {"target", rest_name(1)}}});
        return t.size() >= 2;
    }
    if (c == "rename") {
        if (t.size() < 3) return false;
        ops = nlohmann::json::array({{{"op", "rename"}, {"target", t[1]}, {"to", t[2]}}});
        return true;
    }
    if (c == "loopcut" || c == "loop-cut" || (c == "loop" && t.size() >= 2 && lower_copy(t[1]) == "cut")) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "loop_cut";
        size_t i = (c == "loop") ? 2 : 1;
        for (; i < t.size(); ++i) {
            std::string k = lower_copy(t[i]);
            if (k == "x" || k == "y" || k == "z" || k == "vertical" || k == "horizontal") op["axis"] = k;
            else if (is_region(k)) op["region"] = k;
            else if (is_number(t[i])) op["cuts"] = int(to_f(t[i]));
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "bevel") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "bevel";
        for (size_t i = 1; i < t.size(); ++i) {
            std::string k = lower_copy(t[i]);
            if (is_region(k)) op["region"] = k;
            else if (is_number(t[i])) op["amount"] = to_f(t[i]);
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "subdivide" || c == "subdiv") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "subdivide";
        for (size_t i = 1; i < t.size(); ++i) {
            std::string k = lower_copy(t[i]);
            if (is_region(k)) op["region"] = k;
            else if (is_number(t[i])) op["cuts"] = int(to_f(t[i]));
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "merge") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "merge";
        for (size_t i = 1; i < t.size(); ++i) {
            std::string k = lower_copy(t[i]);
            if (is_region(k)) op["region"] = k;
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "fill") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "fill";
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "mirror") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "mirror";
        for (size_t i = 1; i < t.size(); ++i) {
            std::string k = lower_copy(t[i]);
            if (k == "x" || k == "y" || k == "z" || k == "+x" || k == "+y" || k == "+z") op["axis"] = k;
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "solidify" || c == "thicken" || c == "shell") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "solidify";
        for (size_t i = 1; i < t.size(); ++i)
            if (is_number(t[i])) op["thickness"] = to_f(t[i]);
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "array") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "array";
        Vec3 off;
        for (size_t i = 1; i < t.size(); ++i) {
            std::string k = lower_copy(t[i]);
            if (k == "x" || k == "y" || k == "z") op["axis"] = k;
            else if (parse_vec3_tokens(t, i, off)) {
                op["offset"] = vecj(off);
                i += 2;
            } else if (is_number(t[i])) {
                if (!op.contains("count")) op["count"] = int(to_f(t[i]));
                else op["spacing"] = to_f(t[i]);
            }
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "spin") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "spin";
        for (size_t i = 1; i < t.size(); ++i) {
            std::string k = lower_copy(t[i]);
            if (k == "x" || k == "y" || k == "z") op["axis"] = k;
            else if (is_number(t[i])) {
                if (!op.contains("steps")) op["steps"] = int(to_f(t[i]));
                else op["angle"] = to_f(t[i]);
            }
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "bridge") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "bridge";
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "join") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "join";
        if (t.size() >= 3) {
            op["target"] = t[1];
            op["other"] = t[2];
        } else if (t.size() >= 2)
            op["other"] = t[1];
        ops = nlohmann::json::array({op});
        return t.size() >= 2;
    }
    if (c == "ground" || (c == "to" && t.size() >= 2 && lower_copy(t[1]) == "ground")) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "ground";
        if (c == "ground" && t.size() >= 2 && !is_number(t[1])) op["target"] = t[1];
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "origin") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "origin";
        if (t.size() >= 2 && lower_copy(t[1]) != "to" && lower_copy(t[1]) != "geometry")
            op["target"] = t[1];
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "triangulate" || c == "tris") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "triangulate"}})});
        return true;
    }
    if (c == "poke") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "poke"}})});
        return true;
    }
    if (c == "flip") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "flip"}})});
        return true;
    }
    if (c == "dissolve") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "dissolve"}})});
        return true;
    }
    if (c == "separate") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "separate"}})});
        return true;
    }
    if (c == "weld") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "weld";
        for (size_t i = 1; i < t.size(); ++i)
            if (is_number(t[i])) op["distance"] = to_f(t[i]);
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "smooth") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "smooth";
        for (size_t i = 1; i < t.size(); ++i)
            if (is_number(t[i])) {
                if (!op.contains("factor")) op["factor"] = to_f(t[i]);
                else op["iters"] = int(to_f(t[i]));
            }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "push") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "push";
        for (size_t i = 1; i < t.size(); ++i)
            if (is_number(t[i])) op["amount"] = to_f(t[i]);
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "sphere" || (c == "to" && t.size() >= 2 && lower_copy(t[1]) == "sphere")) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "sphere";
        for (size_t i = 1; i < t.size(); ++i)
            if (is_number(t[i])) op["factor"] = to_f(t[i]);
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "flatten") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "flatten";
        for (size_t i = 1; i < t.size(); ++i) {
            std::string k = lower_copy(t[i]);
            if (k == "x" || k == "y" || k == "z") op["axis"] = k;
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "randomize") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "randomize";
        for (size_t i = 1; i < t.size(); ++i)
            if (is_number(t[i])) op["amount"] = to_f(t[i]);
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "apply") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "apply";
        if (t.size() >= 2) op["target"] = t[1];
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "bisect") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "bisect";
        for (size_t i = 1; i < t.size(); ++i) {
            std::string k = lower_copy(t[i]);
            if (k == "x" || k == "y" || k == "z") op["axis"] = k;
            else if (is_number(t[i])) op["distance"] = to_f(t[i]);
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "snap") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "snap";
        for (size_t i = 1; i < t.size(); ++i)
            if (is_number(t[i])) op["grid"] = to_f(t[i]);
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "split") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "split"}})});
        return true;
    }
    if (c == "connect") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "connect"}})});
        return true;
    }
    if (c == "grow") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "grow"}})});
        return true;
    }
    if (c == "invert") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "invert"}})});
        return true;
    }
    if (c == "linked") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "linked"}})});
        return true;
    }
    if (c == "rig") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "rig";
        op["kind"] = t.size() >= 2 ? lower_copy(t[1]) : "biped";
        if (t.size() >= 3) op["pose"] = lower_copy(t[2]);
        if (t.size() >= 2 && (lower_copy(t[1]) == "tpose" || lower_copy(t[1]) == "apose")) {
            op["kind"] = "biped";
            op["pose"] = lower_copy(t[1]);
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "bind") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "bind"}})});
        return true;
    }
    if (c == "pose") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "pose";
        Vec3 v;
        if (t.size() >= 2 && !is_number(t[1])) op["target"] = t[1];
        size_t i = op.contains("target") ? 2 : 1;
        if (parse_vec3_tokens(t, i, v)) op["rotation"] = vecj(v);
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "rest") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "rest"}})});
        return true;
    }
    if (c == "shape") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "shape";
        if (t.size() >= 2 && !is_number(t[1])) op["name"] = t[1];
        for (size_t i = 1; i < t.size(); ++i)
            if (is_number(t[i])) op["value"] = to_f(t[i]);
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "lod") {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "lod";
        if (t.size() >= 2 && is_number(t[1])) op["level"] = int(to_f(t[1]));
        if (t.size() >= 3 && is_number(t[2])) op["levels"] = int(to_f(t[2]));
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "describe") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "describe"}})});
        return true;
    }
    if (c == "select" && t.size() >= 2 && lower_copy(t[1]) == "linked") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "linked"}})});
        return true;
    }
    if (c == "select" && t.size() >= 3 && lower_copy(t[1]) == "faces" && lower_copy(t[2]) == "more") {
        ops = nlohmann::json::array({nlohmann::json::object({{"op", "grow"}})});
        return true;
    }
    if (c == "select" && t.size() >= 2 &&
        (lower_copy(t[1]) == "verts" || lower_copy(t[1]) == "vert" || lower_copy(t[1]) == "vertices" ||
         lower_copy(t[1]) == "vertex")) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "select_verts";
        if (t.size() >= 3 && !is_number(t[2])) op["region"] = lower_copy(t[2]);
        else {
            nlohmann::json vs = nlohmann::json::array();
            for (size_t i = 2; i < t.size(); ++i)
                if (is_number(t[i])) vs.push_back(int(to_f(t[i])));
            if (!vs.empty()) op["verts"] = vs;
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "select" && t.size() >= 2 &&
        (lower_copy(t[1]) == "edges" || lower_copy(t[1]) == "edge")) {
        nlohmann::json op = nlohmann::json::object();
        std::string third = t.size() >= 3 ? lower_copy(t[2]) : "";
        if (third == "loop") {
            op["op"] = "select_edge_loop";
            if (t.size() >= 4) {
                if (is_region(lower_copy(t[3]))) op["region"] = lower_copy(t[3]);
                else op["axis"] = t[3];
            }
        } else if (third == "ring") {
            op["op"] = "select_edge_ring";
            if (t.size() >= 4) op["axis"] = t[3];
        } else {
            op["op"] = "select_edges";
            if (t.size() >= 3 && is_region(third)) op["region"] = third;
            else if (t.size() >= 3) op["axis"] = third;
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "select" && t.size() >= 2 && (lower_copy(t[1]) == "faces" || lower_copy(t[1]) == "face")) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "select_faces";
        if (t.size() >= 3 && !is_number(t[2])) op["region"] = lower_copy(t[2]);
        else {
            nlohmann::json fs = nlohmann::json::array();
            for (size_t i = 2; i < t.size(); ++i)
                if (is_number(t[i])) fs.push_back(int(to_f(t[i])));
            op["faces"] = fs;
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "select") {
        ops = nlohmann::json::array({{{"op", "select"}, {"target", rest_name(1)}}});
        return t.size() >= 2;
    }
    if (c == "move" || c == "pos") {
        Vec3 v;
        if (t.size() >= 5 && parse_vec3_tokens(t, 2, v)) {
            ops = nlohmann::json::array({{{"op", "move"}, {"target", t[1]}, {"translation", vecj(v)}}});
            return true;
        }
        return false;
    }
    if (c == "translate" || c == "shift" || c == "grab") {
        Vec3 v;
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "translate";
        if (t.size() >= 5 && !is_number(t[1]) && parse_vec3_tokens(t, 2, v)) {
            op["target"] = t[1];
            op["delta"] = vecj(v);
            ops = nlohmann::json::array({op});
            return true;
        }
        size_t i = 1;
        if (i < t.size() && (lower_copy(t[i]) == "verts" || lower_copy(t[i]) == "vertices")) ++i;
        if (parse_vec3_tokens(t, i, v)) {
            op["delta"] = vecj(v);
            ops = nlohmann::json::array({op});
            return true;
        }
        return false;
    }
    if (c == "rotate") {
        Vec3 v;
        if (t.size() >= 5 && parse_vec3_tokens(t, 2, v)) {
            ops = nlohmann::json::array({{{"op", "rotate"}, {"target", t[1]}, {"rotation", vecj(v)}}});
            return true;
        }
        return false;
    }
    if (c == "scale") {
        Vec3 v;
        if (t.size() >= 5 && parse_vec3_tokens(t, 2, v)) {
            ops = nlohmann::json::array({{{"op", "scale"}, {"target", t[1]}, {"scale", vecj(v)}}});
            return true;
        }
        if (t.size() == 3 && is_number(t[2])) {
            float f = to_f(t[2]);
            ops = nlohmann::json::array({{{"op", "scale"}, {"target", t[1]}, {"scale", vecj({f, f, f})}}});
            return true;
        }
        return false;
    }
    if (c == "color" || c == "paint" || c == "material") {
        if (t.size() < 3) return false;
        nlohmann::json op = {{"op", "color"}, {"target", t[1]}};
        Vec3 col;
        if (named_color(t[2], col)) op["albedo"] = vecj(col);
        else {
            Vec3 v;
            if (!parse_vec3_tokens(t, 2, v)) return false;
            op["albedo"] = vecj(v);
        }
        ops = nlohmann::json::array({op});
        return true;
    }
    if (c == "roughness" && t.size() >= 3) {
        ops = nlohmann::json::array({{{"op", "color"}, {"target", t[1]}, {"roughness", to_f(t[2])}}});
        return true;
    }
    if (c == "metallic" && t.size() >= 3) {
        ops = nlohmann::json::array({{{"op", "color"}, {"target", t[1]}, {"metallic", to_f(t[2])}}});
        return true;
    }
    if (c == "parent" && t.size() >= 3) {
        ops = nlohmann::json::array({{{"op", "parent"}, {"target", t[1]}, {"parent", t[2]}}});
        return true;
    }
    if (c == "lookat" || c == "camera") {
        Vec3 eye, center;
        if (t.size() >= 7 && parse_vec3_tokens(t, 1, eye) && parse_vec3_tokens(t, 4, center)) {
            ops = nlohmann::json::array({{{"op", "lookat"}, {"eye", vecj(eye)}, {"center", vecj(center)}}});
            return true;
        }
        if (c == "camera" && t.size() >= 2) {
            ops = nlohmann::json::array({{{"op", "create"}, {"primitive", "camera"}, {"name", t[1]}}});
            return true;
        }
        return false;
    }
    if (c == "frame" || c == "focus") {
        // frame / frame <name> / frame <name> top
        nlohmann::json op = {{"op", "frame"}};
        if (t.size() >= 2 && t[1] != "all" && t[1] != "scene") op["target"] = t[1];
        if (t.size() >= 3) op["axis"] = t.back();
        ops = nlohmann::json::array({op});
        return true;
    }
    return false;
}

struct ColorWord {
    const char* n;
    Vec3 c;
};

bool nl_intent(const std::string& line, Document& doc, nlohmann::json& ops) {
    std::string s = lower_copy(line);
    Vec3 col{0.72f, 0.73f, 0.76f};
    bool has_color = false;
    static const ColorWord kColors[] = {
        {"red", {0.82f, 0.18f, 0.14f}},     {"orange", {0.90f, 0.45f, 0.16f}},
        {"yellow", {0.93f, 0.80f, 0.22f}},  {"green", {0.28f, 0.62f, 0.32f}},
        {"blue", {0.22f, 0.42f, 0.82f}},    {"cyan", {0.22f, 0.72f, 0.78f}},
        {"purple", {0.52f, 0.28f, 0.72f}},  {"magenta", {0.78f, 0.22f, 0.58f}},
        {"pink", {0.90f, 0.48f, 0.62f}},    {"white", {0.92f, 0.92f, 0.94f}},
        {"black", {0.08f, 0.08f, 0.09f}},   {"gray", {0.55f, 0.56f, 0.58f}},
        {"grey", {0.55f, 0.56f, 0.58f}},    {"gold", {0.83f, 0.65f, 0.22f}},
        {"silver", {0.75f, 0.76f, 0.80f}},  {"brown", {0.45f, 0.28f, 0.16f}},
        {"kiln", {0.88f, 0.42f, 0.18f}},    {"ember", {0.85f, 0.28f, 0.12f}},
    };
    std::string color_word;
    for (auto& cw : kColors) {
        if (s.find(cw.n) != std::string::npos) {
            col = cw.c;
            has_color = true;
            color_word = cw.n;
            break;
        }
    }

    auto find_existing = [&]() -> std::string {
        for (const auto& n : doc.scene.nodes) {
            if (s.find(lower_copy(n.name)) != std::string::npos) return n.name;
        }
        return {};
    };

    PrimType prim;
    bool has_prim = false;
    const char* prim_words[] = {"cube", "box", "sphere", "ball", "plane", "ground", "floor",
                                "cylinder", "tube", "cone", "torus", "donut", "doughnut"};
    std::string prim_tok;
    for (const char* w : prim_words) {
        if (s.find(w) != std::string::npos) {
            parse_prim(w, prim);
            prim_tok = w;
            has_prim = true;
            break;
        }
    }

    Vec3 size{1, 1, 1};
    if (s.find("tiny") != std::string::npos) size = {0.25f, 0.25f, 0.25f};
    else if (s.find("small") != std::string::npos) size = {0.5f, 0.5f, 0.5f};
    else if (s.find("large") != std::string::npos || s.find("big") != std::string::npos) size = {2.5f, 2.5f, 2.5f};
    else if (s.find("huge") != std::string::npos) size = {6, 6, 6};

    bool create_like = s.find("add") != std::string::npos || s.find("create") != std::string::npos ||
                       s.find("make") != std::string::npos || s.find("spawn") != std::string::npos ||
                       s.find("put") != std::string::npos || s.find("place") != std::string::npos;

    if (create_like && (s.find("camera") != std::string::npos)) {
        ops = nlohmann::json::array({{{"op", "create"}, {"primitive", "camera"}}});
        return true;
    }
    if (create_like && (s.find("sun") != std::string::npos || s.find("light") != std::string::npos)) {
        std::string p = s.find("point") != std::string::npos ? "point" : "sun";
        ops = nlohmann::json::array({{{"op", "create"}, {"primitive", p}}});
        return true;
    }

    if (has_prim && (create_like || doc.scene.nodes.empty() || s.find("a ") != std::string::npos ||
                     s.find("an ") != std::string::npos)) {
        if (prim == PrimType::Plane) {
            size = {size.x * 4, 1, size.z * 4};
            if (s.find("ground") != std::string::npos || s.find("floor") != std::string::npos) size = {12, 1, 12};
        }
        nlohmann::json op = {{"op", "create"}, {"primitive", prim_name(prim)}, {"size", vecj(size)}};
        if (has_color) op["albedo"] = vecj(col);
        if (prim != PrimType::Plane) {
            op["translation"] = vecj({0, size.y * 0.5f, 0});
        }
        ops = nlohmann::json::array({op});
        if (s.find("on a") != std::string::npos && s.find("plane") == std::string::npos &&
            s.find("ground") != std::string::npos) {
            ops.push_back({{"op", "create"},
                           {"primitive", "plane"},
                           {"name", "Ground"},
                           {"size", {12, 1, 12}},
                           {"albedo", {0.42, 0.43, 0.45}}});
        }
        return true;
    }

    std::string target = find_existing();
    if (target.empty() && doc.scene.selection >= 0 &&
        doc.scene.selection < static_cast<int>(doc.scene.nodes.size()))
        target = doc.scene.nodes[doc.scene.selection].name;

    auto region_in = [&]() -> std::string {
        const char* rs[] = {"top", "bottom", "left", "right", "front", "back"};
        for (const char* r : rs)
            if (s.find(r) != std::string::npos) return r;
        return {};
    };
    auto amount_in = [&](float d) {
        auto by = s.find(" by ");
        if (by != std::string::npos) {
            auto rest = split_ws(s.substr(by + 1));
            if (rest.size() >= 2 && is_number(rest[1])) return to_f(rest[1]);
        }
        auto toks = split_ws(s);
        for (auto it = toks.rbegin(); it != toks.rend(); ++it)
            if (is_number(*it)) return to_f(*it);
        return d;
    };

    std::string region = region_in();
    if (s.find("loop cut") != std::string::npos || s.find("loopcut") != std::string::npos ||
        s.find("cut loop") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "loop_cut";
        op["cuts"] = std::max(1, int(amount_in(1)));
        if (s.find("vertical") != std::string::npos || s.find(" y") != std::string::npos) op["axis"] = "y";
        else if (s.find("horizontal") != std::string::npos) op["axis"] = "x";
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("bevel") != std::string::npos || s.find("chamfer") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "bevel";
        op["amount"] = amount_in(0.12f);
        if (!region.empty()) op["region"] = region;
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("subdivide") != std::string::npos || s.find("subdiv") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "subdivide";
        op["cuts"] = std::max(1, int(amount_in(1)));
        if (!region.empty()) op["region"] = region;
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("merge") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "merge";
        if (!region.empty()) op["region"] = region;
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("weld") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "weld";
        op["distance"] = amount_in(0.001f);
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("tpose") != std::string::npos || s.find("t-pose") != std::string::npos ||
        s.find("t pose") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "rig";
        op["kind"] = "biped";
        op["pose"] = "tpose";
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("apose") != std::string::npos || s.find("a-pose") != std::string::npos ||
        s.find("a pose") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "rig";
        op["kind"] = "biped";
        op["pose"] = "apose";
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("quadruped") != std::string::npos || s.find("animal rig") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "rig";
        op["kind"] = "quadruped";
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("face rig") != std::string::npos || s.find("facial") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "rig";
        op["kind"] = "face";
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("bind") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "bind";
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("fill") != std::string::npos || s.find("cap the") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "fill";
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("mirror") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "mirror";
        if (s.find(" y") != std::string::npos || s.find("vertical") != std::string::npos) op["axis"] = "y";
        else if (s.find(" z") != std::string::npos) op["axis"] = "z";
        else op["axis"] = "x";
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("solidify") != std::string::npos || s.find("thicken") != std::string::npos ||
        s.find("shell") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "solidify";
        op["thickness"] = amount_in(0.08f);
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("array") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "array";
        op["count"] = std::max(2, int(amount_in(3)));
        if (s.find(" y") != std::string::npos) op["axis"] = "y";
        else if (s.find(" z") != std::string::npos) op["axis"] = "z";
        else op["axis"] = "x";
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("spin") != std::string::npos || s.find("revolve") != std::string::npos ||
        s.find("lathe") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "spin";
        op["steps"] = std::max(2, int(amount_in(8)));
        if (s.find(" x") != std::string::npos) op["axis"] = "x";
        else if (s.find(" z") != std::string::npos) op["axis"] = "z";
        else op["axis"] = "y";
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("bridge") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "bridge";
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("join") != std::string::npos && !target.empty()) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "join";
        auto toks = split_ws(s);
        std::string other;
        for (size_t i = 0; i < toks.size(); ++i) {
            if (lower_copy(toks[i]) == "and" && i + 1 < toks.size()) other = toks[i + 1];
            if (lower_copy(toks[i]) == "join" && i + 1 < toks.size() && other.empty()) {
                int f = doc.scene.find_node(toks[i + 1]);
                if (f >= 0 && toks[i + 1] != target) other = toks[i + 1];
            }
        }
        if (other.empty()) return false;
        op["other"] = other;
        if (!target.empty()) op["target"] = target;
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("ground") != std::string::npos || s.find("sit on") != std::string::npos ||
        s.find("floor it") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "ground";
        if (!target.empty()) op["target"] = target;
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("origin") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "origin";
        if (!target.empty()) op["target"] = target;
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("edge loop") != std::string::npos || s.find("loop of edges") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "select_edge_loop";
        if (!region.empty()) op["region"] = region;
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("edge ring") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "select_edge_ring";
        ops = nlohmann::json::array({op});
        return true;
    }

    if (s.find("extrude") != std::string::npos || s.find("pull") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "extrude";
        op["amount"] = amount_in(0.4f);
        if (!region.empty()) op["region"] = region;
        ops = nlohmann::json::array({op});
        return true;
    }
    if (s.find("inset") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "inset";
        op["amount"] = amount_in(0.15f);
        if (!region.empty()) op["region"] = region;
        ops = nlohmann::json::array({op});
        return true;
    }
    if ((s.find("select") != std::string::npos) &&
        (s.find("vert") != std::string::npos || s.find("vertex") != std::string::npos)) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "select_verts";
        if (!target.empty()) op["target"] = target;
        if (!region.empty()) op["region"] = region;
        ops = nlohmann::json::array({op});
        return true;
    }
    if ((s.find("select") != std::string::npos) &&
        (s.find("face") != std::string::npos || !region.empty())) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "select_faces";
        if (!target.empty()) op["target"] = target;
        if (!region.empty()) op["region"] = region;
        ops = nlohmann::json::array({op});
        return true;
    }

    if (!target.empty() && has_color &&
        (s.find("color") != std::string::npos || s.find("paint") != std::string::npos ||
         s.find("make it") != std::string::npos || s.find("make the") != std::string::npos)) {
        ops = nlohmann::json::array({{{"op", "color"}, {"target", target}, {"albedo", vecj(col)}}});
        return true;
    }
    if ((s.find("delete") != std::string::npos || s.find("remove") != std::string::npos) &&
        s.find("face") != std::string::npos) {
        nlohmann::json op = nlohmann::json::object();
        op["op"] = "delete_faces";
        if (!region.empty()) op["region"] = region;
        ops = nlohmann::json::array({op});
        return true;
    }
    if (!target.empty() && (s.find("delete") != std::string::npos || s.find("remove") != std::string::npos)) {
        ops = nlohmann::json::array({{{"op", "delete"}, {"target", target}}});
        return true;
    }
    if (!target.empty() && (s.find("duplicate") != std::string::npos || s.find("copy") != std::string::npos)) {
        ops = nlohmann::json::array({{{"op", "duplicate"}, {"target", target}}});
        return true;
    }
    if (!target.empty() && s.find("metal") != std::string::npos) {
        ops = nlohmann::json::array(
            {{{"op", "color"}, {"target", target}, {"metallic", 1.0}, {"roughness", 0.25}}});
        return true;
    }
    if (s.find("undo") != std::string::npos) {
        ops = nlohmann::json::array({{{"op", "undo"}}});
        return true;
    }
    if (s.find("redo") != std::string::npos) {
        ops = nlohmann::json::array({{{"op", "redo"}}});
        return true;
    }
    if (s.find("new scene") != std::string::npos || s == "reset" || s == "clear scene") {
        ops = nlohmann::json::array({{{"op", "new"}}});
        return true;
    }
    return false;
}

AgentReply call_llm(const std::string& input, Document& doc, const AgentConfig& cfg) {
    AgentReply r;
    if (cfg.api_key.empty()) {
        r.ok = false;
        r.text = "I couldn't parse that. Set KILN_API_KEY for the language model, or use DSL (try `help`).";
        return r;
    }
    nlohmann::json body;
    body["model"] = cfg.model;
    body["temperature"] = 0.1;
    std::string scene_summary = "Scene '" + doc.scene.name + "' objects:";
    if (doc.scene.nodes.empty()) scene_summary += " (empty)";
    for (const auto& n : doc.scene.nodes) {
        scene_summary += " " + n.name + "(" + node_type_name(n.type) + ")";
    }
    body["messages"] = nlohmann::json::array({
        {{"role", "system"}, {"content", kSystemPrompt}},
        {{"role", "user"}, {"content", scene_summary + "\n\nRequest: " + input}}
    });
    std::string url = cfg.api_base;
    if (!url.empty() && url.back() == '/') url.pop_back();
    url += "/chat/completions";
    std::string cmd = "curl -sS --max-time 60 " + shell_quote(url) +
                      " -H 'Content-Type: application/json' -H " +
                      shell_quote(std::string("Authorization: Bearer ") + cfg.api_key) + " -d " +
                      shell_quote(body.dump());
    std::string out = run_curl(cmd);
    try {
        auto j = nlohmann::json::parse(out);
        if (j.contains("error")) {
            r.ok = false;
            r.text = "LLM error: " + j["error"].dump();
            return r;
        }
        std::string content = j["choices"][0]["message"]["content"].get<std::string>();
        nlohmann::json ops;
        if (!extract_json_array(content, ops)) {
            r.ok = false;
            r.text = "LLM did not return ops JSON.\n" + content;
            return r;
        }
        return apply_and_reply(doc, ops);
    } catch (const std::exception& e) {
        r.ok = false;
        r.text = std::string("LLM parse failed: ") + e.what() + "\n" + out.substr(0, 400);
        return r;
    }
}

}  // namespace

bool named_color(const std::string& name, Vec3& out) {
    static const std::unordered_map<std::string, Vec3> k{
        {"red", {0.82f, 0.18f, 0.14f}},    {"orange", {0.90f, 0.45f, 0.16f}},
        {"yellow", {0.93f, 0.80f, 0.22f}}, {"green", {0.28f, 0.62f, 0.32f}},
        {"blue", {0.22f, 0.42f, 0.82f}},   {"cyan", {0.22f, 0.72f, 0.78f}},
        {"purple", {0.52f, 0.28f, 0.72f}}, {"magenta", {0.78f, 0.22f, 0.58f}},
        {"pink", {0.90f, 0.48f, 0.62f}},   {"white", {0.92f, 0.92f, 0.94f}},
        {"black", {0.08f, 0.08f, 0.09f}},  {"gray", {0.55f, 0.56f, 0.58f}},
        {"grey", {0.55f, 0.56f, 0.58f}},   {"gold", {0.83f, 0.65f, 0.22f}},
        {"silver", {0.75f, 0.76f, 0.80f}}, {"brown", {0.45f, 0.28f, 0.16f}},
        {"kiln", {0.88f, 0.42f, 0.18f}},   {"ember", {0.85f, 0.28f, 0.12f}},
    };
    auto it = k.find(lower_copy(name));
    if (it == k.end()) return false;
    out = it->second;
    return true;
}

AgentConfig agent_config_from_env() {
    AgentConfig c;
    if (const char* k = std::getenv("KILN_API_KEY")) c.api_key = k;
    else if (const char* k2 = std::getenv("OPENAI_API_KEY")) c.api_key = k2;
    if (const char* b = std::getenv("KILN_API_BASE")) c.api_base = b;
    else if (const char* b2 = std::getenv("OPENAI_BASE_URL")) c.api_base = b2;
    if (const char* m = std::getenv("KILN_MODEL")) c.model = m;
    return c;
}

std::string help_text() {
    return R"(kiln — Linux AI-native 3D modeler

Speak to it, or use the ops language:

  add a red cube
  create cube Box size 1 1 1 at 0 0.5 0
  color Box kiln
  move Box 0 1 0
  rotate Box 0 45 0
  scale Box 1.5
  create plane Ground size 12 1 12
  lookat 6 4 7  0 0.5 0
  frame Box            (aim the camera at something by name)
  frame all top        (frame the whole scene from above)
  select faces top
  extrude 0.5
  inset 0.12
  delete faces bottom
  select edges top
  select edge loop
  loopcut 1
  loopcut y
  bevel 0.12
  select verts top
  grab 0 0.2 0
  subdivide 1
  merge
  fill
  mirror x
  solidify 0.08
  array 4 x 1.4
  spin 8
  bridge
  join Sphere
  ground
  origin
  triangulate / poke / flip / dissolve
  duplicate faces / separate
  weld 0.001 / smooth 0.5 / push 0.1
  sphere / flatten y / randomize 0.05
  apply / bisect y / snap 0.1
  split / connect / grow / invert / linked
  rig biped tpose
  rig biped apose
  rig quadruped
  rig face
  bind
  pose UpperArm.L 0 0 40
  rest
  shape smile 1
  lod
  lod 1
  describe
  duplicate Box
  delete Box
  export scene.gltf
  preview scene.png
  samples 4            (supersampling for previews, 1..16, default 2)
  shadow [N]           (contact shadows, 0..4, default 0 = off)
  ao [N]               (ambient occlusion, 0..4, default 0 = off)
  glbpreview on|off|WxH (bake a preview PNG into an exported GLB, off by default)
  undo / redo / new
  list / info
  help

Y is up. Without KILN_API_KEY, English is a small built-in agent.
With KILN_API_KEY (OpenAI-compatible), any modeling request becomes ops.

Machine interface: run `kiln --rpc` for JSON-RPC 2.0 on stdin/stdout, so an
external agent process can send ops and read structured results back. The same
ops the REPL accepts are reachable there, plus render/export with --samples.)";
}

AgentReply interpret(const std::string& input, Document& doc, const AgentConfig& cfg) {
    std::string line = trim(input);
    AgentReply r;
    if (line.empty()) {
        r.ok = false;
        r.text = "empty";
        return r;
    }
    if (lower_copy(line) == "help" || line == "?") {
        r.text = help_text();
        return r;
    }

    if (line[0] == '[' || line[0] == '{') {
        try {
            auto j = nlohmann::json::parse(line);
            return apply_and_reply(doc, j);
        } catch (const std::exception& e) {
            r.ok = false;
            r.text = std::string("json: ") + e.what();
            return r;
        }
    }

    nlohmann::json ops;
    if (dsl_command(line, ops)) return apply_and_reply(doc, ops);
    if (nl_intent(line, doc, ops)) return apply_and_reply(doc, ops);
    return call_llm(line, doc, cfg);
}

}  // namespace kiln
