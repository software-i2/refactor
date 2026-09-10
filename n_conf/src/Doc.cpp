// Copyright by BeeX [2026]

#include <n_conf/Doc.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace conf {
namespace {

const double kNoNumber = std::numeric_limits<double>::quiet_NaN();

// Scalars and one-level lists are leaves; maps recurse. A list stays whole so a
// four-element window reads as one key rather than four.
void flatten(const YAML::Node &node,
             const std::string &prefix,
             std::map<std::string, std::string> &leaf,
             std::map<std::string, std::vector<std::string>> &list) {

    if (node.IsMap()) {
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            const std::string key = it->first.Scalar();
            flatten(it->second, prefix.empty() ? key : prefix + "." + key, leaf, list);
        }
        return;
    }

    if (node.IsSequence()) {
        std::vector<std::string> items;
        for (std::size_t i = 0; i < node.size(); ++i) {
            items.push_back(node[i].Scalar());
        }
        list[prefix] = items;
        return;
    }

    leaf[prefix] = node.Scalar();
}

bool toDouble(const std::string &text, double &out) {
    char       *end = NULL;
    const double v  = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || !std::isfinite(v)) {
        return false;
    }
    out = v;
    return true;
}

}  // namespace

const char *const Doc::JOINT[4] = {"base", "shoulder", "elbow", "wrist"};

bool Doc::load(const std::string &path,
               const std::vector<std::string> &own,
               const std::vector<std::string> &borrow) {
    leaf_.clear();
    list_.clear();
    read_.clear();
    own_.clear();
    problems_.clear();

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception &e) {
        problems_.push_back("cannot read " + path + ": " + e.what());
        return false;
    }
    if (!root.IsMap()) {
        problems_.push_back(path + " is not a mapping of sections");
        return false;
    }

    std::vector<std::string> all = own;
    all.insert(all.end(), borrow.begin(), borrow.end());
    own_.insert(own.begin(), own.end());

    for (std::size_t s = 0; s < all.size(); ++s) {
        const std::string &name = all[s];
        if (!root[name]) {
            problems_.push_back("section '" + name + "' is missing from " + path);
            continue;
        }
        flatten(root[name], name, leaf_, list_);
    }
    return problems_.empty();
}

// A key belongs to an owned section when its first dotted part is one.
bool Doc::owned(const std::string &key) const {
    const std::string::size_type dot = key.find('.');
    return own_.find(dot == std::string::npos ? key : key.substr(0, dot)) != own_.end();
}

const std::string *Doc::find(const std::string &key) {
    read_.insert(key);
    const std::map<std::string, std::string>::const_iterator it = leaf_.find(key);
    if (it == leaf_.end()) {
        problems_.push_back(key + " is missing");
        return NULL;
    }
    return &it->second;
}

double Doc::num(const std::string &key) {
    const std::string *text = find(key);
    if (text == NULL) {
        return kNoNumber;
    }
    double v = 0.0;
    if (!toDouble(*text, v)) {
        problems_.push_back(key + " is '" + *text + "', which is not a finite number");
        return kNoNumber;
    }
    return v;
}

int Doc::integer(const std::string &key) {
    const double v = num(key);
    if (!std::isfinite(v) || v != std::floor(v)) {
        if (std::isfinite(v)) {
            problems_.push_back(key + " must be a whole number");
        }
        return std::numeric_limits<int>::min();
    }
    return static_cast<int>(v);
}

bool Doc::flag(const std::string &key) {
    const std::string *text = find(key);
    if (text == NULL) {
        return false;
    }
    if (*text == "true" || *text == "True" || *text == "1") {
        return true;
    }
    if (*text == "false" || *text == "False" || *text == "0") {
        return false;
    }
    problems_.push_back(key + " is '" + *text + "', which is not true or false");
    return false;
}

std::string Doc::text(const std::string &key) {
    const std::string *found = find(key);
    return found == NULL ? std::string() : *found;
}

void Doc::nums(const std::string &key, double *out, size_t count) {
    read_.insert(key);
    const std::map<std::string, std::vector<std::string> >::const_iterator it = list_.find(key);
    if (it == list_.end()) {
        problems_.push_back(key + " is missing, or is not a list");
        return;
    }
    if (it->second.size() != count) {
        problems_.push_back(key + " needs " + std::to_string(count) + " numbers, has "
                            + std::to_string(it->second.size()));
        return;
    }
    for (size_t i = 0; i < count; ++i) {
        double v = 0.0;
        if (!toDouble(it->second[i], v)) {
            problems_.push_back(key + "[" + std::to_string(i) + "] is '" + it->second[i]
                                + "', which is not a finite number");
            return;
        }
        out[i] = v;
    }
}

void Doc::perJoint(const std::string &key, double *out) {
    for (int j = 0; j < 4; ++j) {
        const double v = num(key + "." + JOINT[j]);
        if (std::isfinite(v)) {
            out[j] = v;
        }
    }
}

std::vector<std::string> Doc::unused() const {
    std::vector<std::string> left;
    for (std::map<std::string, std::string>::const_iterator it = leaf_.begin();
         it != leaf_.end(); ++it) {
        if (owned(it->first) && read_.find(it->first) == read_.end()) {
            left.push_back(it->first);
        }
    }
    for (std::map<std::string, std::vector<std::string> >::const_iterator it = list_.begin();
         it != list_.end(); ++it) {
        if (owned(it->first) && read_.find(it->first) == read_.end()) {
            left.push_back(it->first);
        }
    }
    std::sort(left.begin(), left.end());
    return left;
}

std::string Doc::effective() const {
    std::string out;
    for (std::set<std::string>::const_iterator it = read_.begin(); it != read_.end(); ++it) {
        const std::map<std::string, std::string>::const_iterator leaf = leaf_.find(*it);
        if (leaf != leaf_.end()) {
            out += "  " + *it + ": " + leaf->second + "\n";
            continue;
        }
        const std::map<std::string, std::vector<std::string> >::const_iterator lst = list_.find(*it);
        if (lst == list_.end()) {
            continue;  // read but absent; report() already names it
        }
        out += "  " + *it + ": [";
        for (std::size_t i = 0; i < lst->second.size(); ++i) {
            out += (i ? ", " : "") + lst->second[i];
        }
        out += "]\n";
    }
    return out;
}

std::string Doc::report() const {
    std::string out;
    for (std::size_t i = 0; i < problems_.size(); ++i) {
        out += "  " + problems_[i] + "\n";
    }

    const std::vector<std::string> left = unused();
    for (std::size_t i = 0; i < left.size(); ++i) {
        out += "  " + left[i] + " is set but nothing reads it -- a typo, or left over\n";
    }
    return out;
}

}  // namespace conf
