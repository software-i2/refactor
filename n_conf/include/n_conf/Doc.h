// Copyright by BeeX [2026]

#ifndef N_CONF_DOC_H
#define N_CONF_DOC_H

#include <cstddef>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace conf {

// The config file, read strictly. There are no defaults anywhere in this tree:
// every tunable comes from the yaml, a missing key is an error, and a key
// nothing reads is an error too -- that is what catches a typo, which a
// defaulted read never can.
//
// Problems accumulate rather than throwing at the first one, so one run tells
// you everything wrong with the file instead of one thing per run.
class Doc {
public:
    // `own` sections must be read completely: anything left over in one is a
    // typo or a stale key, and is reported. `borrow` sections are readable but
    // belong to another node, so unread keys there are that node's business.
    // Sections outside both are not loaded at all.
    bool load(const std::string &path,
              const std::vector<std::string> &own,
              const std::vector<std::string> &borrow = std::vector<std::string>());

    // Every getter names a full dotted key: "ctrl.rate_hz". Missing or
    // unconvertible records a problem and returns a poison value.
    double      num(const std::string &key);
    int         integer(const std::string &key);
    bool        flag(const std::string &key);
    std::string text(const std::string &key);

    // Fixed-length list of numbers. Wrong length is a problem, and `out` is left
    // alone so it keeps its poison.
    void nums(const std::string &key, double *out, size_t count);

    // Same, keyed by joint name rather than position, so a four-element list
    // cannot be silently written down in the wrong order.
    static const char *const JOINT[4];  // base, shoulder, elbow, wrist
    void perJoint(const std::string &key, double *out);

    // False when anything was missing, malformed, or went unread.
    bool ok() const { return problems_.empty() && unused().empty(); }

    // Missing keys, wrong types, bad lengths. Empty does not mean ok(): an
    // unused key is a problem too, and lives in unused().
    const std::vector<std::string> &problems() const { return problems_; }

    // Keys in an owned section that nothing asked for.
    std::vector<std::string> unused() const;

    // Every problem and every unused key, one per line, ready to log.
    std::string report() const;

    // Every key that was actually read, with the value it was read as, one per
    // line. Logged at startup so a bad run is diagnosable from the log alone
    // rather than by guessing which file the node picked up.
    std::string effective() const;

private:
    const std::string *find(const std::string &key);

    bool owned(const std::string &key) const;

    std::map<std::string, std::string> leaf_;   // dotted key -> scalar text
    std::map<std::string, std::vector<std::string>> list_;
    std::set<std::string>              read_;
    std::set<std::string>              own_;
    std::vector<std::string>           problems_;
};

}  // namespace conf

#endif  // N_CONF_DOC_H
